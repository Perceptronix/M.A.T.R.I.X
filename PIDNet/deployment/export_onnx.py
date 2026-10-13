"""Export the trained PIDNet-S RELLIS-3D checkpoint to ONNX."""

from pathlib import Path
import sys

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from models.pidnet import PIDNet


NUM_CLASSES = 19
INPUT_HEIGHT = 512
INPUT_WIDTH = 1024
CHECKPOINT = ROOT / "output" / "rellis" / "pidnet_small_rellis" / "best.pt"
OUTPUT = Path(__file__).resolve().parent / "model.onnx"


def load_state_dict(path: Path) -> dict:
    checkpoint = torch.load(path, map_location="cpu")
    if isinstance(checkpoint, dict) and "state_dict" in checkpoint:
        checkpoint = checkpoint["state_dict"]
    elif isinstance(checkpoint, dict) and "model" in checkpoint:
        checkpoint = checkpoint["model"]

    if not isinstance(checkpoint, dict):
        raise TypeError(f"Unsupported checkpoint format: {type(checkpoint).__name__}")

    prefixes = ("module.", "model.")
    state_dict = {}
    for key, value in checkpoint.items():
        clean_key = key
        for prefix in prefixes:
            if clean_key.startswith(prefix):
                clean_key = clean_key[len(prefix):]
        state_dict[clean_key] = value
    return state_dict


def build_model() -> torch.nn.Module:
    model = PIDNet(
        m=2,
        n=3,
        num_classes=NUM_CLASSES,
        planes=32,
        ppm_planes=96,
        head_planes=128,
        augment=False,
    )
    missing, unexpected = model.load_state_dict(load_state_dict(CHECKPOINT), strict=False)
    if missing:
        raise RuntimeError(f"Checkpoint is missing inference keys: {missing}")
    allowed_training_keys = ("seghead_p.", "seghead_d.", "sem_loss.")
    unexpected = [
        key for key in unexpected
        if not key.startswith(allowed_training_keys)
    ]
    if unexpected:
        raise RuntimeError(f"Unexpected checkpoint keys: {unexpected}")
    return model.eval()


def main() -> None:
    if not CHECKPOINT.is_file():
        raise FileNotFoundError(CHECKPOINT)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)

    model = build_model()
    example = torch.zeros(1, 3, INPUT_HEIGHT, INPUT_WIDTH, dtype=torch.float32)
    with torch.no_grad():
        expected = model(example)

    torch.onnx.export(
        model,
        example,
        str(OUTPUT),
        input_names=["input"],
        output_names=["logits"],
        opset_version=17,
        dynamo=False,
    )

    import onnx
    import onnxruntime as ort

    onnx_model = onnx.load(str(OUTPUT))
    onnx.checker.check_model(onnx_model)
    session = ort.InferenceSession(str(OUTPUT), providers=["CPUExecutionProvider"])
    actual = session.run(["logits"], {"input": example.numpy()})[0]
    expected_np = expected.detach().numpy()
    max_abs = float(np.max(np.abs(expected_np - actual)))
    if not np.allclose(expected_np, actual, rtol=1e-3, atol=1e-4):
        raise RuntimeError(f"ONNX mismatch: max_abs_diff={max_abs}")

    print(f"Exported: {OUTPUT}")
    print(f"Input: {tuple(example.shape)} {example.dtype}")
    print(f"Output: {tuple(actual.shape)} float32")
    print(f"PyTorch/ONNX max_abs_diff: {max_abs:.8g}")


if __name__ == "__main__":
    main()