# PIDNet-S RELLIS-3D deployment

`model.onnx` is exported from `output/rellis/pidnet_small_rellis/best.pt` by:

```powershell
python deployment/export_onnx.py
```

The exporter constructs the trained PIDNet-S architecture (`m=2`, `n=3`,
`planes=32`, `ppm_planes=96`, `head_planes=128`, `augment=False`), loads the
checkpoint, checks for missing or unexpected state-dict keys, validates the
ONNX graph, and compares one zero-valued input with PyTorch using ONNX Runtime.

## Contract

- Input: `input`, `float32`, shape `(1, 3, 512, 1024)` in `NCHW` order.
- Preprocessing: read BGR with OpenCV, convert BGR to RGB, resize to width
  `1024` and height `512` with linear interpolation, divide by `255.0`, then
  normalize channels with mean `(0.485, 0.456, 0.406)` and standard deviation
  `(0.229, 0.224, 0.225)`.
- Output: `logits`, `float32`, shape `(1, 19, 64, 128)`. The spatial output is
  at the model's native 1/8 resolution.
- Full-resolution output: bilinearly upsample logits to `(512, 1024)` before
  postprocessing when a full-resolution map is required.

Postprocessing is `softmax(logits, axis=1)`, followed by `argmax(axis=1)` to
produce the integer `class_map`. The confidence map is the maximum softmax
probability over the class axis: `max(softmax(logits), axis=1)`.

## Entropy gate

PIDNet must be called only when the upstream terrain entropy assessment says
terrain is high-entropy. This repository does not define or train an entropy
threshold, so deployment integration must supply that decision and record its
chosen threshold externally. No threshold or accuracy/performance claim is
implied by this export.
