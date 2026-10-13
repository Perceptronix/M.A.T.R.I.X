"""
Recovery state machine.

Tracks DWA failures and lack of forward progress, and escalates through a
bounded sequence of actions instead of ever spinning or driving blindly
forever:

  stall (DWA fails, or no progress for `stall_timeout`)
    -> STOP, request a replan
  still stalled after the replan
    -> alternate ROCK (short forward/reverse pulses) and ROTATE in place,
       then request another replan
  recovery attempts exceeded `max_recovery_attempts`
    -> FAILED (caller stops and reports failure, does not retry forever)

ROCK was added after Rubicon-terrain physics testing (Part 2-4 of the
terrain-navigation phase): a stalled vehicle nose-on against a rock face
made essentially zero net progress under sustained constant-velocity
forward driving in every tested heading, but a real off-road recovery
technique -- alternating short forward and reverse pulses -- produced
measured, repeatable forward progress (over 1 m net, climbing 0.7 m) from
the same stuck position with no instability. ROTATE alone (the original
recovery action) doesn't address this failure mode: it rescans the area
but does nothing to break a linear wedge against an obstacle. Attempts
alternate rock first (attempt 1, 3, 5...) then rotate (attempt 2, 4, 6...)
since rock is what actually resolved the measured stuck case, with rotate
still available as a fallback (e.g. for a genuinely blocked heading where
no amount of pulsing would help, but a different heading might).
"""


class RecoveryManager:

    def __init__(self, stall_timeout=4.0, progress_threshold=0.25,
                 rotate_duration=2.5,
                 # Raised from an initial 4: with rock_duration=6.0 and
                 # max_recovery_attempts=4 (2 rock windows), a live
                 # autonomous run made barely more net progress than doing
                 # nothing. With rock_duration=20.0 the same budget (2 rock
                 # windows = 40s) produced real, measured progress (dist to
                 # goal 5.39 -> 4.49 m) but still exhausted before fully
                 # clearing the gap -- the manual test that established
                 # this technique needed ~40-60 s of cumulative pulsing
                 # (~30+ cycles) through this exact spot. Raised to 6 (3
                 # rock windows = 60s) to match that budget. Still clearly
                 # bounded, never infinite.
                 # Raised again, 6 -> 10: a live run with 6 (3 rock
                 # windows) made steady, real, monotonic progress every
                 # single cycle (dist to goal 5.39 -> 4.22 m, ~1.17 m
                 # advance) right up to the moment it exhausted its
                 # budget, with no plateau or regression and DWA still
                 # returning fully feasible commands at the final tick --
                 # that trend, not a fixed intuition, is the basis for
                 # extending further rather than accepting a
                 # still-improving run as a hard stop.
                 max_recovery_attempts=10,
                 rock_duration=20.0):
        self.stall_timeout = stall_timeout
        self.progress_threshold = progress_threshold
        self.rotate_duration = rotate_duration
        self.max_recovery_attempts = max_recovery_attempts
        self.rock_duration = rock_duration

        self._last_progress_time = None
        self._last_progress_dist = None
        self._recovery_attempts = 0
        self._rotate_until = None
        self._rock_until = None
        self._dwa_fail_streak = 0

    def reset_progress_clock(self, now, dist_to_goal):
        self._last_progress_time = now
        self._last_progress_dist = dist_to_goal

    def note_dwa_result(self, feasible_count):
        if feasible_count == 0:
            self._dwa_fail_streak += 1
        else:
            self._dwa_fail_streak = 0

    def is_rotating(self, now):
        return self._rotate_until is not None and now < self._rotate_until

    def is_rocking(self, now):
        return self._rock_until is not None and now < self._rock_until

    def start_rotate(self, now):
        self._rotate_until = now + self.rotate_duration

    def start_rock(self, now):
        self._rock_until = now + self.rock_duration

    def check(self, now, dist_to_goal):
        """
        Advance the recovery state machine one control tick.

        Returns one of: 'ok', 'stall_replan', 'rock', 'rocking', 'rotate',
        'rotating', 'failed'.
        """
        if self.is_rocking(now):
            return 'rocking'
        if self._rock_until is not None and now >= self._rock_until:
            self._rock_until = None
            self.reset_progress_clock(now, dist_to_goal)
            return 'stall_replan'

        if self.is_rotating(now):
            return 'rotating'
        if self._rotate_until is not None and now >= self._rotate_until:
            self._rotate_until = None
            self.reset_progress_clock(now, dist_to_goal)
            return 'stall_replan'

        if self._last_progress_time is None:
            self.reset_progress_clock(now, dist_to_goal)
            return 'ok'

        made_progress = (self._last_progress_dist - dist_to_goal) > self.progress_threshold
        if made_progress:
            self.reset_progress_clock(now, dist_to_goal)
            self._dwa_fail_streak = 0
            return 'ok'

        stalled_too_long = (now - self._last_progress_time) > self.stall_timeout
        dwa_stuck = self._dwa_fail_streak >= 5

        if stalled_too_long or dwa_stuck:
            self._recovery_attempts += 1
            if self._recovery_attempts > self.max_recovery_attempts:
                return 'failed'
            self.reset_progress_clock(now, dist_to_goal)
            self._dwa_fail_streak = 0
            if self._recovery_attempts % 2 == 1:
                self.start_rock(now)
                return 'rock'
            self.start_rotate(now)
            return 'rotate'

        return 'ok'
