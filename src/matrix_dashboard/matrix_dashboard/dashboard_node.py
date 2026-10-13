#!/usr/bin/env python3
"""M.A.T.R.I.X engineering/mission dashboard.

Runs on the system python3 (no torch/cv2 needed -- uses PIL, already
present system-wide, only to re-encode the raw camera Image into a JPEG
for the browser). Every value it displays comes from a real subscribed
ROS 2 topic; nothing is simulated, incremented on a timer, or invented.
The event log only records lines when a subscribed value actually
changes (e.g. a gate-state transition), each stamped with wall-clock
time at the moment that ROS message was received.

Architecture: a background thread runs rclpy.spin(); the main thread runs
a stdlib http.server. State is shared through one dict guarded by a lock.
This keeps the dashboard to stdlib + PIL, avoiding Flask/FastAPI/
websockets, per "keep it simple and robust, do not over-engineer".
"""
import io
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32, String, Int32, Bool, Float32MultiArray
from geometry_msgs.msg import Twist

try:
    from PIL import Image as PILImage
    HAVE_PIL = True
except ImportError:
    HAVE_PIL = False

STATE_LOCK = threading.Lock()
STATE = {
    'sim_started_at': time.time(),
    'odom': None,
    'state_estimated': None,
    'cmd_vel': None,
    'lidar_hz_count': 0,
    'lidar_last_stamp': 0.0,
    'lidar_rate': 0.0,
    'camera_hz_count': 0,
    'camera_last_stamp': 0.0,
    'camera_rate': 0.0,
    'entropy': None,
    'gate_decision': None,
    'semantic_trigger': None,
    'semantic_gate_state': None,
    'semantic_inference_count': None,
    'semantic_confidence': None,
    'semantic_entropy': None,
    'semantic_cost_bias': None,
    'semantic_dominant_class': None,
    'obstacle_point_count': None,
    'costmap_summary': None,
    'elevation_summary': None,
    'roughness_summary': None,
    'slope_summary': None,
    'step_height_summary': None,
    'clearance_summary': None,
    'camera_jpeg': None,
    'planner_status': None,
    'plan_waypoints': None,
    'planner_cmd_vel': None,
    'safety_cmd_vel': None,
    'safety_clearance': None,
    'safety_barrier_h': None,
    'safety_intervention_count': None,
    'events': [],
}
MAX_EVENTS = 60


def log_event(msg: str):
    line = f'[{time.strftime("%H:%M:%S")}] {msg}'
    with STATE_LOCK:
        STATE['events'].append(line)
        if len(STATE['events']) > MAX_EVENTS:
            STATE['events'] = STATE['events'][-MAX_EVENTS:]


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def array_summary(data):
    vals = [v for v in data if v == v and math.isfinite(v)]  # drop NaN
    if not vals:
        return {'min': None, 'max': None, 'mean': None, 'n': 0}
    return {
        'min': round(min(vals), 4),
        'max': round(max(vals), 4),
        'mean': round(sum(vals) / len(vals), 4),
        'n': len(vals),
    }


class DashboardNode(Node):
    def __init__(self):
        super().__init__('matrix_dashboard')
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Odometry, '/state/estimated', self.on_state_estimated, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(
            Float32, '/terrain/entropy', self.on_entropy, 10)
        self.create_subscription(String, '/perception/gate_decision', self.on_gate_decision, 10)
        self.create_subscription(Bool, '/perception/semantic_trigger', self.on_semantic_trigger, 10)
        self.create_subscription(String, '/semantic/gate_state', self.on_semantic_gate_state, 10)
        self.create_subscription(Int32, '/semantic/inference_count', self.on_inference_count, 10)
        self.create_subscription(Float32, '/semantic/confidence', self.on_confidence, 10)
        self.create_subscription(Float32, '/semantic/entropy', self.on_semantic_entropy, 10)
        self.create_subscription(Float32, '/semantic/cost_bias', self.on_cost_bias, 10)
        self.create_subscription(String, '/semantic/dominant_class', self.on_dominant_class, 10)
        self.create_subscription(
            OccupancyGrid, '/terrain/costmap', self.on_costmap, 10)
        self.create_subscription(
            Float32MultiArray, '/terrain/elevation', self.on_elevation, 10)
        self.create_subscription(
            Float32MultiArray, '/terrain/roughness', self.on_roughness, 10)
        self.create_subscription(
            Float32MultiArray, '/terrain/slope', self.on_slope, 10)
        self.create_subscription(
            Float32MultiArray, '/terrain/step_height', self.on_step_height, 10)
        self.create_subscription(
            Float32MultiArray, '/terrain/clearance', self.on_clearance, 10)
        from sensor_msgs.msg import PointCloud2
        self.create_subscription(
            PointCloud2, '/perception/obstacles', self.on_obstacles, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/lidar/points', self.on_lidar, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/left/image_raw', self.on_camera, qos_profile_sensor_data)

        from nav_msgs.msg import Path
        self.create_subscription(String, '/planner/global_status', self.on_planner_status, 10)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(Twist, '/planner/cmd_vel', self.on_planner_cmd_vel, 10)
        self.create_subscription(Twist, '/safety/cmd_vel', self.on_safety_cmd_vel, 10)
        self.create_subscription(Float32, '/safety/clearance', self.on_safety_clearance, 10)
        self.create_subscription(Float32, '/safety/barrier_h', self.on_safety_h, 10)
        self.create_subscription(
            Float32, '/safety/intervention_count', self.on_safety_intervention_count, 10)

        self._last_gate_decision = None
        self._last_semantic_gate_state = None
        self._last_inference_count = None
        self._last_planner_status = None
        log_event('Dashboard node started, subscribing to live topics')

    def on_planner_status(self, msg: String):
        if msg.data != self._last_planner_status:
            log_event(f'Planner (Hybrid A*) -> {msg.data}')
            self._last_planner_status = msg.data
        with STATE_LOCK:
            STATE['planner_status'] = msg.data

    def on_plan(self, msg):
        with STATE_LOCK:
            STATE['plan_waypoints'] = len(msg.poses)

    def on_planner_cmd_vel(self, msg: Twist):
        with STATE_LOCK:
            STATE['planner_cmd_vel'] = {
                'linear_x': round(msg.linear.x, 3), 'angular_z': round(msg.angular.z, 3)}

    def on_safety_cmd_vel(self, msg: Twist):
        with STATE_LOCK:
            STATE['safety_cmd_vel'] = {
                'linear_x': round(msg.linear.x, 3), 'angular_z': round(msg.angular.z, 3)}

    def on_safety_clearance(self, msg: Float32):
        with STATE_LOCK:
            STATE['safety_clearance'] = round(float(msg.data), 3)

    def on_safety_h(self, msg: Float32):
        with STATE_LOCK:
            STATE['safety_barrier_h'] = round(float(msg.data), 3)

    def on_safety_intervention_count(self, msg: Float32):
        with STATE_LOCK:
            STATE['safety_intervention_count'] = int(msg.data)

    def on_odom(self, msg: Odometry):
        with STATE_LOCK:
            STATE['odom'] = {
                'x': round(msg.pose.pose.position.x, 3),
                'y': round(msg.pose.pose.position.y, 3),
                'z': round(msg.pose.pose.position.z, 3),
                'yaw': round(yaw_from_quaternion(msg.pose.pose.orientation), 3),
                'linear_x': round(msg.twist.twist.linear.x, 3),
                'angular_z': round(msg.twist.twist.angular.z, 3),
            }

    def on_state_estimated(self, msg: Odometry):
        with STATE_LOCK:
            STATE['state_estimated'] = {
                'x': round(msg.pose.pose.position.x, 3),
                'y': round(msg.pose.pose.position.y, 3),
                'yaw': round(yaw_from_quaternion(msg.pose.pose.orientation), 3),
            }

    def on_cmd_vel(self, msg: Twist):
        with STATE_LOCK:
            STATE['cmd_vel'] = {
                'linear_x': round(msg.linear.x, 3),
                'angular_z': round(msg.angular.z, 3),
            }

    def on_entropy(self, msg: Float32):
        with STATE_LOCK:
            STATE['entropy'] = round(float(msg.data), 5)

    def on_gate_decision(self, msg: String):
        if msg.data != self._last_gate_decision:
            log_event(f'Gate decision -> {msg.data}')
            self._last_gate_decision = msg.data
        with STATE_LOCK:
            STATE['gate_decision'] = msg.data

    def on_semantic_trigger(self, msg: Bool):
        with STATE_LOCK:
            STATE['semantic_trigger'] = bool(msg.data)

    def on_semantic_gate_state(self, msg: String):
        if msg.data != self._last_semantic_gate_state:
            log_event(f'PIDNet gate -> {msg.data}')
            self._last_semantic_gate_state = msg.data
        with STATE_LOCK:
            STATE['semantic_gate_state'] = msg.data

    def on_inference_count(self, msg: Int32):
        if self._last_inference_count is not None and msg.data != self._last_inference_count:
            log_event(f'PIDNet inference #{msg.data}')
        self._last_inference_count = msg.data
        with STATE_LOCK:
            STATE['semantic_inference_count'] = int(msg.data)

    def on_confidence(self, msg: Float32):
        with STATE_LOCK:
            STATE['semantic_confidence'] = round(float(msg.data), 4)

    def on_semantic_entropy(self, msg: Float32):
        with STATE_LOCK:
            STATE['semantic_entropy'] = round(float(msg.data), 4)

    def on_cost_bias(self, msg: Float32):
        with STATE_LOCK:
            STATE['semantic_cost_bias'] = round(float(msg.data), 4)

    def on_dominant_class(self, msg: String):
        with STATE_LOCK:
            STATE['semantic_dominant_class'] = msg.data

    def on_costmap(self, msg: OccupancyGrid):
        vals = [v for v in msg.data if v != -1]
        with STATE_LOCK:
            STATE['costmap_summary'] = {
                'min': min(vals) if vals else None,
                'max': max(vals) if vals else None,
                'mean': round(sum(vals) / len(vals), 2) if vals else None,
                'unknown_cells': len(msg.data) - len(vals),
                'total_cells': len(msg.data),
                'frame': msg.header.frame_id,
            }

    def on_elevation(self, msg: Float32MultiArray):
        with STATE_LOCK:
            STATE['elevation_summary'] = array_summary(msg.data)

    def on_roughness(self, msg: Float32MultiArray):
        with STATE_LOCK:
            STATE['roughness_summary'] = array_summary(msg.data)

    def on_slope(self, msg: Float32MultiArray):
        with STATE_LOCK:
            STATE['slope_summary'] = array_summary(msg.data)

    def on_step_height(self, msg: Float32MultiArray):
        with STATE_LOCK:
            STATE['step_height_summary'] = array_summary(msg.data)

    def on_clearance(self, msg: Float32MultiArray):
        with STATE_LOCK:
            STATE['clearance_summary'] = array_summary(msg.data)

    def on_obstacles(self, msg):
        with STATE_LOCK:
            STATE['obstacle_point_count'] = msg.width * msg.height

    def on_lidar(self, msg):
        now = time.time()
        with STATE_LOCK:
            last = STATE['lidar_last_stamp']
            if last > 0:
                dt = now - last
                if dt > 0:
                    STATE['lidar_rate'] = round(1.0 / dt, 2)
            STATE['lidar_last_stamp'] = now
            STATE['lidar_hz_count'] += 1

    def on_camera(self, msg: Image):
        now = time.time()
        with STATE_LOCK:
            last = STATE['camera_last_stamp']
            if last > 0:
                dt = now - last
                if dt > 0:
                    STATE['camera_rate'] = round(1.0 / dt, 2)
            STATE['camera_last_stamp'] = now
            STATE['camera_hz_count'] += 1
        # Throttle JPEG re-encode to ~2 Hz to protect sim performance.
        if now - getattr(self, '_last_jpeg_time', 0.0) < 0.5:
            return
        self._last_jpeg_time = now
        if not HAVE_PIL or msg.encoding != 'rgb8':
            return
        try:
            img = PILImage.frombytes('RGB', (msg.width, msg.height), bytes(msg.data))
            buf = io.BytesIO()
            img.save(buf, format='JPEG', quality=70)
            with STATE_LOCK:
                STATE['camera_jpeg'] = buf.getvalue()
        except Exception as exc:  # noqa: BLE001 - dashboard must not crash the demo
            self.get_logger().warn(f'camera jpeg encode failed: {exc}')


HTML_PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>M.A.T.R.I.X Dashboard</title>
<style>
body{font-family:monospace;background:#0b0f14;color:#d7e0e8;margin:0;padding:12px;}
h1{color:#7fd1ff;font-size:18px;margin:0 0 12px 0;}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:10px;}
.card{background:#111820;border:1px solid #24313f;border-radius:6px;padding:10px;}
.card h2{font-size:13px;color:#7fd1ff;margin:0 0 8px 0;text-transform:uppercase;}
.row{display:flex;justify-content:space-between;font-size:13px;padding:2px 0;}
.row span:first-child{color:#8fa3b3;}
.LOW,.MATH_ONLY,.INACTIVE{color:#5fd97a;font-weight:bold;}
.HIGH,.SEMANTIC_REQUIRED,.ACTIVE{color:#ff9d5c;font-weight:bold;}
#log{background:#0d1319;border:1px solid #24313f;border-radius:6px;padding:8px;
     height:160px;overflow-y:auto;font-size:12px;grid-column:1/-1;}
#cam{width:100%;max-width:320px;border:1px solid #24313f;border-radius:4px;}
.notavail{color:#5a6b7a;font-style:italic;}
</style></head>
<body>
<h1>M.A.T.R.I.X &mdash; Member 1+2+3 Live Dashboard (real ROS 2 data only)</h1>
<div class="grid" id="grid"></div>
<script>
function row(label, value, cls) {
  return `<div class="row"><span>${label}</span><span class="${cls||''}">${value}</span></div>`;
}
function fmt(v, digits) {
  if (v === null || v === undefined) return '<span class="notavail">n/a</span>';
  if (typeof v === 'number') return v.toFixed(digits === undefined ? 3 : digits);
  return v;
}
async function tick() {
  let s;
  try { s = await (await fetch('/api/state')).json(); } catch(e) { return; }
  const o = s.odom || {};
  const se = s.state_estimated || {};
  const cv = s.cmd_vel || {};
  const cm = s.costmap_summary || {};
  let html = '';

  html += `<div class="card"><h2>Mission Status</h2>
    ${row('Simulation','RUNNING')}
    ${row('UGV', (o.linear_x||0)!==0 ? 'MOVING':'STOPPED')}
    ${row('GPS','OFF (by design)')}
    ${row('Mission time (s)', s.mission_time_s, 1)}
    </div>`;

  html += `<div class="card"><h2>Robot State (/odom)</h2>
    ${row('X (m)', fmt(o.x))} ${row('Y (m)', fmt(o.y))} ${row('Z (m)', fmt(o.z))}
    ${row('Yaw (rad)', fmt(o.yaw))}
    ${row('Linear vel (m/s)', fmt(o.linear_x))}
    ${row('Angular vel (rad/s)', fmt(o.angular_z))}
    </div>`;

  html += `<div class="card"><h2>State Estimation (/state/estimated)</h2>
    ${se.x!==undefined ? row('X (m)', fmt(se.x))+row('Y (m)', fmt(se.y))+row('Yaw (rad)', fmt(se.yaw))
      : row('Status','<span class=notavail>NOT PUBLISHING</span>')}
    </div>`;

  html += `<div class="card"><h2>Perception</h2>
    ${row('LiDAR rate (Hz)', fmt(s.lidar_rate,2))}
    ${row('Stereo cam rate (Hz)', fmt(s.camera_rate,2))}
    ${row('Obstacle points', s.obstacle_point_count ?? '<span class=notavail>n/a</span>')}
    ${row('Costmap cells (valid/total)', cm.total_cells ? (cm.total_cells-cm.unknown_cells)+'/'+cm.total_cells : '<span class=notavail>n/a</span>')}
    </div>`;

  const el = s.elevation_summary||{}, ro = s.roughness_summary||{}, sl = s.slope_summary||{},
        st = s.step_height_summary||{}, cl = s.clearance_summary||{};
  html += `<div class="card"><h2>Terrain</h2>
    ${row('Elevation mean (m)', fmt(el.mean))}
    ${row('Roughness mean (m)', fmt(ro.mean))}
    ${row('Slope mean (deg)', fmt(sl.mean))}
    ${row('Step height max (m)', fmt(st.max))}
    ${row('Clearance min (m)', fmt(cl.min))}
    ${row('Costmap cost (min/mean/max)', cm.min!==undefined ? cm.min+' / '+cm.mean+' / '+cm.max : '<span class=notavail>n/a</span>')}
    </div>`;

  const gate = s.gate_decision || 'n/a';
  html += `<div class="card"><h2>Entropy Gate</h2>
    ${row('Entropy [0,1]', fmt(s.entropy,5))}
    ${row('Threshold', '0.30 (gating_supervisor_node default)')}
    ${row('Gate decision', gate, gate)}
    ${row('Semantic trigger', s.semantic_trigger)}
    </div>`;

  const gs = s.semantic_gate_state || 'n/a';
  html += `<div class="card"><h2>Lightweight AI (PIDNet)</h2>
    ${row('Model', 'PIDNet-S (real trained ONNX, RELLIS-3D)')}
    ${row('Semantic validity', '<span class="notavail">limited &mdash; author asserts no accuracy number</span>')}
    ${row('Gate state', gs, gs)}
    ${row('Inference count', s.semantic_inference_count ?? 0)}
    ${row('Confidence', fmt(s.semantic_confidence))}
    ${row('Semantic entropy', fmt(s.semantic_entropy))}
    ${row('Cost bias', fmt(s.semantic_cost_bias))}
    ${row('Dominant class', s.semantic_dominant_class || '<span class=notavail>n/a</span>')}
    </div>`;

  const pv = s.planner_cmd_vel || {}, sv = s.safety_cmd_vel || {};
  html += `<div class="card"><h2>Decision / Movement Response</h2>
    ${row('Final cmd_vel linear.x', fmt(cv.linear_x))}
    ${row('Final cmd_vel angular.z', fmt(cv.angular_z))}
    ${row('DWA proposed (pre-CBF)', pv.linear_x!==undefined ? pv.linear_x+' / '+pv.angular_z : '<span class=notavail>n/a</span>')}
    ${row('CBF output (/safety/cmd_vel)', sv.linear_x!==undefined ? sv.linear_x+' / '+sv.angular_z : '<span class=notavail>n/a</span>')}
    ${row('Source', 'Member 4 (Hybrid A*+DWA) -> Member 5 (CBF) -> /cmd_vel')}
    </div>`;

  html += `<div class="card"><h2>Navigation (Members 4/5, first working implementation)</h2>
    ${row('Hybrid A* status', s.planner_status || '<span class=notavail>n/a</span>', s.planner_status==='GOAL_REACHED'?'ACTIVE':'')}
    ${row('Plan waypoints', s.plan_waypoints ?? '<span class=notavail>n/a</span>')}
    ${row('CBF clearance (m)', fmt(s.safety_clearance))}
    ${row('CBF barrier h', fmt(s.safety_barrier_h))}
    ${row('CBF interventions', s.safety_intervention_count ?? 0)}
    ${row('Scope', '<span class="notavail">receding-horizon local planner; no global map/SLAM exists in this stack</span>')}
    </div>`;

  html += `<div class="card"><h2>Camera (left)</h2>
    <img id="cam" src="/api/camera_left.jpg?t=${Date.now()}"></div>`;

  html += `<div id="log"><b>Event log</b><br>${(s.events||[]).slice().reverse().join('<br>')}</div>`;

  document.getElementById('grid').innerHTML = html;
}
setInterval(tick, 700);
tick();
</script>
</body></html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # keep stdout clean; ROS logging covers this node

    def do_GET(self):
        if self.path.startswith('/api/state'):
            with STATE_LOCK:
                snap = {k: v for k, v in STATE.items() if k != 'camera_jpeg'}
            snap['mission_time_s'] = round(time.time() - STATE['sim_started_at'], 1)
            body = json.dumps(snap).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith('/api/camera_left.jpg'):
            with STATE_LOCK:
                jpeg = STATE['camera_jpeg']
            if jpeg is None:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)
        else:
            body = HTML_PAGE.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    server = ThreadingHTTPServer(('0.0.0.0', 8088), Handler)
    node.get_logger().info('Dashboard serving on http://localhost:8088')
    log_event('Dashboard HTTP server listening on :8088')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
