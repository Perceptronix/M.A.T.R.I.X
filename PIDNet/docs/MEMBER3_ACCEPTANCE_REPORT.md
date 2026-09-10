# MEMBER3 acceptance report

## Scope

- Source checkpoint: `output/rellis/pidnet_small_rellis/best.pt`.
- Export target: `deployment/model.onnx`.
- Retraining: not performed.
- Entropy gate: preserved as a caller-side high-entropy requirement. No
  threshold is present in this repository, so none is claimed here.

## Required checks

The export script performs these checks when run in an environment with a
working PyTorch installation plus `onnx` and `onnxruntime`:

1. Load the checkpoint into the PIDNet-S inference architecture.
2. Fail on missing or unexpected checkpoint keys.
3. Validate the ONNX graph with `onnx.checker`.
4. Run one `float32` input through PyTorch and ONNX Runtime.
5. Compare the complete output tensor with `allclose` and report maximum
   absolute difference.

## Results

The export and ONNX Runtime comparison completed successfully in the
`PyTorch-GPU` environment using ONNX Runtime's CPU execution provider:

```powershell
python deployment/export_onnx.py
```

- ONNX input: `(1, 3, 512, 1024)` `float32`.
- ONNX output: `(1, 19, 64, 128)` `float32`.
- PyTorch/ONNX maximum absolute difference: `2.4497509e-05`.

No accuracy, entropy threshold, or performance number is asserted; those were
not measured by this deployment check.