# MEMBER3 to MEMBER2.4 handoff

MEMBER3 provides optional semantic context to the existing geometric
perception/fusion pipeline. The handoff should carry:

- `class_map`: integer RELLIS training IDs `0..18`;
- `confidence`: maximum softmax probability per pixel;
- the semantic output dimensions and any resize transform used; and
- whether the upstream entropy gate allowed PIDNet to run.

The consumer must not treat an ungated or low-confidence result as equivalent
to deterministic geometry. PIDNet is invoked only for high-entropy terrain;
the repository does not define the entropy estimator or its threshold.

The 19 IDs and names are defined in `docs/MEMBER3_MODEL.md`. This handoff does
not assign traversability costs or invent fusion weights; those remain
downstream integration decisions requiring project-specific validation.