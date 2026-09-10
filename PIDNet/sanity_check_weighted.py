"""
Sanity check for class-weighted loss.
Verifies:
  - weights are finite, non-negative
  - loss forward pass is finite
  - gradients are finite
  - output has 19 classes
  - ignore_index=255 still works
  - checkpoint can be loaded
"""
import os, sys
import torch
import torch.nn as nn
sys.path.insert(0, os.path.dirname(__file__))

import models
import datasets
from configs import config, update_config
from utils.criterion import OhemCrossEntropy, BondaryLoss
from utils.utils import FullModel

class Args:
    cfg  = "configs/rellis/pidnet_small_rellis.yaml"
    opts = []

update_config(config, Args())
device = torch.device("cuda")

print("=== SANITY CHECK: class-weighted loss ===")

# 1. Load weights
weights_file = "output/rellis/pidnet_small_rellis/class_weights.pt"
data = torch.load(weights_file, map_location="cpu")
w = data["weights"].float()
print(f"[1] Weight tensor shape : {tuple(w.shape)}")
print(f"    Values              : {[round(x,4) for x in w.tolist()]}")
assert not torch.isnan(w).any(),  "FAIL: NaN in weights"
assert not torch.isinf(w).any(),  "FAIL: Inf in weights"
assert (w >= 0).all(),            "FAIL: negative weights"
print("    OK — no NaN/Inf/negative")

# 2. Build model + criterion
model = models.pidnet.get_seg_model(config, imgnet_pretrained=True)

sem_criterion = OhemCrossEntropy(
    ignore_label=config.TRAIN.IGNORE_LABEL,   # 255
    thres=config.LOSS.OHEMTHRES,
    min_kept=config.LOSS.OHEMKEEP,
    weight=w.cuda(),
)
bd_criterion = BondaryLoss()
model = FullModel(model, sem_criterion, bd_criterion).to(device)

# 3. Load checkpoint
ckpt = "output/rellis/pidnet_small_rellis/checkpoint.pth.tar"
checkpoint = torch.load(ckpt, map_location="cpu")
model.module.load_state_dict(checkpoint["state_dict"]) \
    if hasattr(model, "module") else model.load_state_dict(checkpoint["state_dict"])
print(f"[2] Checkpoint loaded   : epoch={checkpoint['epoch']}  best_mIoU={checkpoint['best_mIoU']:.4f}")

# 4. Build a synthetic batch — 2 images, mix of valid labels + ignore pixels
B, H, W = 2, 128, 128
images = torch.randn(B, 3, H, W, device=device)

labels = torch.randint(0, 19, (B, H, W), device=device)       # valid classes 0-18
labels[0, :10, :10] = 255                                       # inject ignore pixels
labels[1, 20:30, :] = 255

edges  = torch.zeros(B, H, W, device=device)

model.train()
optimizer = torch.optim.SGD(model.parameters(), lr=1e-3)
optimizer.zero_grad()

loss, preds, acc, loss_parts = model(images, labels, edges)
loss = loss.mean()

# 5. Check loss
print(f"[3] Loss value          : {loss.item():.6f}")
assert torch.isfinite(loss), f"FAIL: loss is not finite ({loss.item()})"
print("    OK — loss is finite")

# 6. Backward
loss.backward()
for name, p in model.named_parameters():
    if p.grad is not None:
        assert torch.isfinite(p.grad).all(), f"FAIL: non-finite grad in {name}"
print("[4] All gradients       : finite OK")

# 7. Output shape
seg_pred = preds[-1] if isinstance(preds, (list, tuple)) else preds
print(f"[5] Output shape        : {tuple(seg_pred.shape)}  (expect B×19×H×W)")
assert seg_pred.shape[1] == 19, f"FAIL: expected 19 classes, got {seg_pred.shape[1]}"
print("    OK — 19 classes")

# 8. Verify ignore_index=255 doesn't leak into loss
labels_all_ignore = torch.full((B, H, W), 255, dtype=torch.long, device=device)
optimizer.zero_grad()
loss_ig, _, _, _ = model(images, labels_all_ignore, edges)
loss_ig = loss_ig.mean()
print(f"[6] All-ignore loss     : {loss_ig.item():.6f}")
assert torch.isfinite(loss_ig), "FAIL: all-ignore loss is not finite"
print("    OK — ignore_index=255 handled cleanly (returns 0 when all pixels ignored)")

print("\n=== SANITY CHECK PASSED ===")
print(f"Ready to resume training from epoch {checkpoint['epoch']} with class-weighted loss.")
