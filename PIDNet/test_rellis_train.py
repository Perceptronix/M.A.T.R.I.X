import torch
import torch.nn as nn
from torch.utils.data import DataLoader

from configs import config, update_config
import models
import datasets

from utils.criterion import OhemCrossEntropy, BondaryLoss
from utils.utils import FullModel


class Args:
    cfg = "configs/rellis/pidnet_small_rellis.yaml"
    opts = []


update_config(config, Args())

device = torch.device("cuda")

print("=== MEMBER 3 TRAINING SMOKE TEST ===")
print("device:", torch.cuda.get_device_name(0))

dataset = datasets.rellis(
    root=config.DATASET.ROOT,
    list_path=config.DATASET.TRAIN_SET,
    num_classes=config.DATASET.NUM_CLASSES,
    multi_scale=config.TRAIN.MULTI_SCALE,
    flip=config.TRAIN.FLIP,
    ignore_label=config.TRAIN.IGNORE_LABEL,
    base_size=config.TRAIN.BASE_SIZE,
    crop_size=(
        config.TRAIN.IMAGE_SIZE[1],
        config.TRAIN.IMAGE_SIZE[0],
    ),
    scale_factor=config.TRAIN.SCALE_FACTOR,
)

print("dataset:", len(dataset))

loader = DataLoader(
    dataset,
    batch_size=2,
    shuffle=False,
    num_workers=0,
    pin_memory=False,
    drop_last=True,
)

images, labels, edges, sizes, names = next(iter(loader))

print("batch image:", tuple(images.shape))
print("batch label:", tuple(labels.shape))
print("batch edge:", tuple(edges.shape))
print("sample:", names[0])

print("label dtype:", labels.dtype)
print("label min:", labels.min().item())
print("label max:", labels.max().item())
print("unique labels:", torch.unique(labels).cpu().tolist())

images = images.to(device, non_blocking=True)
labels = labels.to(device, non_blocking=True)
edges = edges.to(device, non_blocking=True)

model = models.pidnet.get_seg_model(
    config,
    imgnet_pretrained=True,
)

sem_criterion = OhemCrossEntropy(
    ignore_label=config.TRAIN.IGNORE_LABEL,
    thres=config.LOSS.OHEMTHRES,
    min_kept=config.LOSS.OHEMKEEP,
    weight=dataset.class_weights.float(),
)

print(
    "class_weights:",
    type(dataset.class_weights),
    tuple(dataset.class_weights.shape),
    dataset.class_weights.dtype,
)

bd_criterion = BondaryLoss()

model = FullModel(
    model,
    sem_criterion,
    bd_criterion,
).to(device)

model.train()

optimizer = torch.optim.SGD(
    model.parameters(),
    lr=config.TRAIN.LR,
    momentum=config.TRAIN.MOMENTUM,
    weight_decay=config.TRAIN.WD,
    nesterov=config.TRAIN.NESTEROV,
)

optimizer.zero_grad(set_to_none=True)

loss, predictions, pixel_acc, loss_parts = model(
    images,
    labels,
    edges,
)

seg_loss, bd_loss = loss_parts

# Print shapes before reduction
print("loss shape:", tuple(loss.shape))
print("seg_loss shape:", tuple(seg_loss.shape) if hasattr(seg_loss, 'shape') else "scalar")
print("bd_loss shape:", tuple(bd_loss.shape) if hasattr(bd_loss, 'shape') else "scalar")

# Reduce to scalar if loss is not already a scalar
loss = loss.mean()

print("loss:", loss.detach().cpu().item())
print("seg_loss:", float(seg_loss.detach().cpu().mean()))
print("bd_loss:", float(bd_loss.detach().cpu().mean()))
print("pixel_acc:", float(pixel_acc.detach().cpu()))

loss.backward()

optimizer.step()

print("BACKWARD OK")
print("OPTIMIZER STEP OK")
print("GPU MEMORY MB:",
      round(torch.cuda.max_memory_allocated() / 1024**2, 1))
