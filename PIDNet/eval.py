import torch
import cv2
import numpy as np
from pathlib import Path

# ---- CHANGE THESE 3 PATHS ----
IMAGE = r"C:\Users\nande\OneDrive\Desktop\M.A.T.R.I.X\test.jpg"
WEIGHTS = r"C:\Users\nande\OneDrive\Desktop\M.A.T.R.I.X\PIDNet\output\rellis\pidnet_small_rellis\best.pt"
OUTPUT = "segmented.png"

NUM_CLASSES = 19
IGNORE_LABEL = 255


# Use your existing PIDNet implementation
from models.pidnet import PIDNet


def main():
    device = "cuda" if torch.cuda.is_available() else "cpu"

    # Same architecture used during training
    model = PIDNet(
        m=2,
        n=3,
        num_classes=NUM_CLASSES,
        planes=32,
        ppm_planes=96,
        head_planes=128,
        augment=False,
    )

    # Load weights
    checkpoint = torch.load(WEIGHTS, map_location=device)

    if "state_dict" in checkpoint:
        checkpoint = checkpoint["state_dict"]
    elif "model" in checkpoint:
        checkpoint = checkpoint["model"]

    checkpoint = {
        k.replace("module.", ""): v
        for k, v in checkpoint.items()
    }

    model.load_state_dict(checkpoint, strict=False)
    model.to(device)
    model.eval()

    # Read image
    image = cv2.imread(IMAGE)

    if image is None:
        raise FileNotFoundError(f"Cannot read image: {IMAGE}")

    original = image.copy()
    h, w = image.shape[:2]

    # PIDNet input
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = cv2.resize(image, (1024, 512))

    image = image.astype(np.float32) / 255.0

    # ImageNet normalization
    mean = np.array([0.485, 0.456, 0.406])
    std = np.array([0.229, 0.224, 0.225])

    image = (image - mean) / std

    image = torch.from_numpy(image)
    image = image.permute(2, 0, 1).unsqueeze(0)
    image = image.float().to(device)

    # Inference
    with torch.no_grad():
        output = model(image)

        # PIDNet may return multiple outputs
        if isinstance(output, (tuple, list)):
            output = output[0]

        prediction = torch.argmax(output, dim=1)[0]

    # Back to original image size
    prediction = prediction.cpu().numpy().astype(np.uint8)
    prediction = cv2.resize(
        prediction,
        (w, h),
        interpolation=cv2.INTER_NEAREST
    )

    # -----------------------------
    # Create segmentation overlay
    # -----------------------------

    # Random but fixed colors for 19 classes
    rng = np.random.default_rng(42)
    colors = rng.integers(0, 255, (NUM_CLASSES, 3), dtype=np.uint8)

    mask = colors[prediction]

    # Ignore label
    mask[prediction == IGNORE_LABEL] = 0

    overlay = cv2.addWeighted(
        original,
        0.5,
        mask,
        0.5,
        0
    )

    cv2.imwrite(OUTPUT, overlay)

    print("================================")
    print("PIDNet Single Image Evaluation")
    print("================================")
    print(f"Image   : {IMAGE}")
    print(f"Weights : {WEIGHTS}")
    print(f"Device  : {device}")
    print(f"Output  : {OUTPUT}")
    print("Done.")


if __name__ == "__main__":
    main()