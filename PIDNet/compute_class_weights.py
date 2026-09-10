"""
Compute per-class pixel counts and inverse-sqrt class weights
from the RELLIS-3D training split.

Reads every label mask, applies the ID_MAPPING (raw -> train ID),
counts valid pixels per class (ignore_label=255 excluded),
then computes weight[c] = 1/sqrt(freq[c]), normalised so mean weight = 1.

Saves result to:  output/rellis/pidnet_small_rellis/class_weights.pt
Prints a full report.
"""

import os, sys
import numpy as np
import cv2
import torch

sys.path.insert(0, os.path.dirname(__file__))
from datasets.rellis import Rellis

# ── config ────────────────────────────────────────────────────────────────────
DATA_ROOT    = "data"
TRAIN_LST    = "Rellis_3D_image_split/train.lst"
NUM_CLASSES  = 19
IGNORE_LABEL = 255
OUT_DIR      = "output/rellis/pidnet_small_rellis"
OUT_FILE     = os.path.join(OUT_DIR, "class_weights.pt")

CLASS_NAMES = [
    "void", "grass", "mud", "bush", "concrete", "sky",
    "water", "puddle", "dirt", "gravel", "asphalt",
    "building", "log", "person", "fence", "vehicle",
    "object", "pole", "tree trunk",
]

os.makedirs(OUT_DIR, exist_ok=True)

# ── build file list (same as Rellis.__init__) ─────────────────────────────────
manifest = os.path.join(DATA_ROOT, TRAIN_LST)
files = []
with open(manifest) as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        _, lbl_path = line.split()
        files.append(os.path.join(DATA_ROOT, "Rellis-3D", lbl_path))

print(f"Scanning {len(files)} training label masks …")

# ── count pixels ──────────────────────────────────────────────────────────────
counts = np.zeros(NUM_CLASSES, dtype=np.int64)

for i, lbl_path in enumerate(files):
    raw = cv2.imread(lbl_path, cv2.IMREAD_GRAYSCALE)
    if raw is None:
        print(f"  WARNING: could not read {lbl_path}")
        continue

    # apply ID_MAPPING
    converted = np.full(raw.shape, IGNORE_LABEL, dtype=np.int64)
    for raw_id, train_id in Rellis.ID_MAPPING.items():
        converted[raw == raw_id] = train_id

    # count valid pixels per class
    valid = converted[converted != IGNORE_LABEL]
    for c in range(NUM_CLASSES):
        counts[c] += int((valid == c).sum())

    if (i + 1) % 200 == 0:
        print(f"  {i+1}/{len(files)} done …")

total_valid = counts.sum()
print(f"\nTotal valid pixels: {total_valid:,}")

# ── report pixel counts ───────────────────────────────────────────────────────
print(f"\n{'ID':>3}  {'Class':<12}  {'Pixels':>14}  {'Freq':>10}")
print("-" * 46)
for c in range(NUM_CLASSES):
    freq = counts[c] / total_valid if total_valid > 0 else 0.0
    print(f"{c:>3}  {CLASS_NAMES[c]:<12}  {counts[c]:>14,}  {freq:>10.6f}")

# ── compute weights ───────────────────────────────────────────────────────────
freqs = counts.astype(np.float64) / max(total_valid, 1)

weights = np.zeros(NUM_CLASSES, dtype=np.float64)
for c in range(NUM_CLASSES):
    if counts[c] == 0:
        print(f"\n  WARNING: class {c} ({CLASS_NAMES[c]}) has ZERO training pixels — weight set to 0.")
        weights[c] = 0.0
    else:
        weights[c] = 1.0 / np.sqrt(freqs[c])

# normalise so mean of non-zero weights ≈ 1
nonzero_mask = weights > 0
mean_w = weights[nonzero_mask].mean()
weights[nonzero_mask] /= mean_w
# zero-pixel classes stay at 0 so they contribute nothing to the loss

# ── report weights ────────────────────────────────────────────────────────────
print(f"\n{'ID':>3}  {'Class':<12}  {'Pixels':>14}  {'Freq':>10}  {'Weight':>8}")
print("-" * 56)
for c in range(NUM_CLASSES):
    freq = freqs[c]
    print(f"{c:>3}  {CLASS_NAMES[c]:<12}  {counts[c]:>14,}  {freq:>10.6f}  {weights[c]:>8.4f}")

weights_tensor = torch.tensor(weights, dtype=torch.float32)
torch.save({"counts": counts, "weights": weights_tensor}, OUT_FILE)
print(f"\nSaved -> {OUT_FILE}")
print(f"Weights tensor: {weights_tensor.tolist()}")
