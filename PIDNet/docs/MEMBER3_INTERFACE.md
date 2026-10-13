# MEMBER3 PIDNet interface

MEMBER3 consumes the optional semantic segmentation result from PIDNet-S.

## Invocation gate

Run PIDNet only when the upstream terrain entropy assessment classifies the
terrain as high entropy. The current repository contains no entropy estimator
or calibrated threshold. The caller owns that gate; this model export does
not invent one.

## Input

The ONNX input is `input`: `float32`, `(1, 3, 512, 1024)`, `NCHW`.
Preprocessing is BGR image read, BGR-to-RGB conversion, linear resize to
`1024x512`, division by `255.0`, and ImageNet normalization using mean
`[0.485, 0.456, 0.406]` and standard deviation `[0.229, 0.224, 0.225]`.

## Output

The ONNX output is `logits`: `float32`, `(1, 19, 64, 128)`. Apply softmax on
the 19-class axis. `argmax` produces the integer `class_map`; the maximum
softmax value produces the per-pixel `confidence` map. Upsample logits before
softmax if a full-resolution map is needed.

## Handoff

The semantic result is an optional input to traversability fusion. Confidence,
class IDs, and the entropy-gate decision should remain available to the fusion
consumer. This document does not define an unverified threshold or accuracy.