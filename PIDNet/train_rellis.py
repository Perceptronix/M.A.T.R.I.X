"""
PIDNet-S RELLIS-3D training script.
Reuses all existing infrastructure (models, datasets, criterion, utils).
Adds: epoch-wise console reporting, num_workers=0 for Windows,
      configurable epoch cap, and clean checkpoint saving.
"""

import argparse
import os
import sys
import logging
import timeit

import numpy as np
import torch
import torch.nn as nn
import torch.backends.cudnn as cudnn
from tensorboardX import SummaryWriter

# Make repo root importable (mirrors tools/_init_paths.py)
sys.path.insert(0, os.path.dirname(__file__))

import models
import datasets
from configs import config, update_config
from utils.criterion import CrossEntropy, OhemCrossEntropy, BondaryLoss
from utils.function import train, validate
from utils.utils import FullModel


class Args:
    cfg = "configs/rellis/pidnet_small_rellis.yaml"
    opts = []


def main(max_epochs: int = 5):
    update_config(config, Args())

    # ── output dirs ───────────────────────────────────────────────────────────
    output_dir = os.path.join("output", "rellis", "pidnet_small_rellis")
    log_dir    = os.path.join("log",    "rellis", "pidnet_s")
    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(log_dir,    exist_ok=True)

    # ── logging ───────────────────────────────────────────────────────────────
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(message)s",
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler(os.path.join(output_dir, "train.log"), mode="a"),
        ],
    )
    logger = logging.getLogger()

    logger.info(f"=== PIDNet-S RELLIS-3D Training  (max_epochs={max_epochs}) ===")
    logger.info(f"Output dir : {output_dir}")
    logger.info(f"GPU        : {torch.cuda.get_device_name(0)}")

    # ── cudnn ─────────────────────────────────────────────────────────────────
    cudnn.benchmark   = config.CUDNN.BENCHMARK
    cudnn.deterministic = config.CUDNN.DETERMINISTIC
    cudnn.enabled     = config.CUDNN.ENABLED

    gpus = list(config.GPUS)   # [0]

    # ── model ─────────────────────────────────────────────────────────────────
    imgnet = "imagenet" in config.MODEL.PRETRAINED
    model  = models.pidnet.get_seg_model(config, imgnet_pretrained=imgnet)

    # ── datasets ──────────────────────────────────────────────────────────────
    crop_size = (config.TRAIN.IMAGE_SIZE[1], config.TRAIN.IMAGE_SIZE[0])

    train_dataset = datasets.rellis(
        root=config.DATASET.ROOT,
        list_path=config.DATASET.TRAIN_SET,
        num_classes=config.DATASET.NUM_CLASSES,
        multi_scale=config.TRAIN.MULTI_SCALE,
        flip=config.TRAIN.FLIP,
        ignore_label=config.TRAIN.IGNORE_LABEL,
        base_size=config.TRAIN.BASE_SIZE,
        crop_size=crop_size,
        scale_factor=config.TRAIN.SCALE_FACTOR,
    )

    test_size = (config.TEST.IMAGE_SIZE[1], config.TEST.IMAGE_SIZE[0])
    val_dataset = datasets.rellis(
        root=config.DATASET.ROOT,
        list_path=config.DATASET.TEST_SET,
        num_classes=config.DATASET.NUM_CLASSES,
        multi_scale=False,
        flip=False,
        ignore_label=config.TRAIN.IGNORE_LABEL,
        base_size=config.TEST.BASE_SIZE,
        crop_size=test_size,
    )

    batch_size = config.TRAIN.BATCH_SIZE_PER_GPU * len(gpus)

    # num_workers=0: required on Windows to avoid multiprocessing spawn issues
    trainloader = torch.utils.data.DataLoader(
        train_dataset,
        batch_size=batch_size,
        shuffle=config.TRAIN.SHUFFLE,
        num_workers=0,
        pin_memory=False,
        drop_last=True,
    )
    valloader = torch.utils.data.DataLoader(
        val_dataset,
        batch_size=1,           # keep GPU memory low for 6 GB card
        shuffle=False,
        num_workers=0,
        pin_memory=False,
    )

    logger.info(f"Train samples : {len(train_dataset)}  ({len(trainloader)} batches/epoch)")
    logger.info(f"Val   samples : {len(val_dataset)}")

    # ── criterion ─────────────────────────────────────────────────────────────
    if config.LOSS.USE_OHEM:
        sem_criterion = OhemCrossEntropy(
            ignore_label=config.TRAIN.IGNORE_LABEL,
            thres=config.LOSS.OHEMTHRES,
            min_kept=config.LOSS.OHEMKEEP,
            weight=train_dataset.class_weights,
        )
    else:
        sem_criterion = CrossEntropy(
            ignore_label=config.TRAIN.IGNORE_LABEL,
            weight=train_dataset.class_weights,
        )

    bd_criterion = BondaryLoss()

    model = FullModel(model, sem_criterion, bd_criterion)
    model = nn.DataParallel(model, device_ids=gpus).cuda()

    # ── optimizer ─────────────────────────────────────────────────────────────
    optimizer = torch.optim.SGD(
        model.parameters(),
        lr=config.TRAIN.LR,
        momentum=config.TRAIN.MOMENTUM,
        weight_decay=config.TRAIN.WD,
        nesterov=config.TRAIN.NESTEROV,
    )

    # ── tensorboard ───────────────────────────────────────────────────────────
    writer_dict = {
        "writer": SummaryWriter(log_dir),
        "train_global_steps": 0,
        "valid_global_steps": 0,
    }

    epoch_iters = len(train_dataset) // batch_size
    # num_iters drives the LR poly schedule; use full 100-epoch horizon
    # so LR decay matches the intended schedule even during the 5-epoch run
    num_iters   = config.TRAIN.END_EPOCH * epoch_iters

    best_mIoU   = 0.0
    last_epoch  = 0
    metrics_log = []   # (epoch, val_loss, val_mIoU, gpu_mb)

    # ── resume from checkpoint if available ───────────────────────────────────
    ckpt_path = os.path.join(output_dir, "checkpoint.pth.tar")
    if os.path.isfile(ckpt_path):
        checkpoint = torch.load(ckpt_path, map_location="cpu")
        last_epoch = checkpoint["epoch"]
        best_mIoU  = checkpoint["best_mIoU"]
        model.module.load_state_dict(checkpoint["state_dict"])
        optimizer.load_state_dict(checkpoint["optimizer"])
        logger.info(f"Resumed from checkpoint at epoch {last_epoch}, best_mIoU={best_mIoU:.4f}")

    logger.info(f"epoch_iters={epoch_iters}  num_iters={num_iters}")
    logger.info(f"LR={config.TRAIN.LR}  batch={batch_size}  ignore={config.TRAIN.IGNORE_LABEL}")
    logger.info(f"Starting from epoch {last_epoch}, running to epoch {max_epochs}")

    t0 = timeit.default_timer()

    for epoch in range(last_epoch, max_epochs):
        # ── train one epoch ───────────────────────────────────────────────────
        train(
            config, epoch, config.TRAIN.END_EPOCH,
            epoch_iters, config.TRAIN.LR, num_iters,
            trainloader, optimizer, model, writer_dict,
        )

        # ── checkpoint after train (before validate, so no crash loses progress)
        torch.save(
            {
                "epoch": epoch + 1,
                "best_mIoU": best_mIoU,
                "state_dict": model.module.state_dict(),
                "optimizer": optimizer.state_dict(),
            },
            ckpt_path,
        )
        logger.info(f"Checkpoint saved -> {ckpt_path}")

        # ── validate ──────────────────────────────────────────────────────────
        val_loss, mean_IoU, IoU_array = validate(
            config, valloader, model, writer_dict,
        )

        gpu_mb = round(torch.cuda.max_memory_allocated() / 1024**2, 1)

        logger.info(
            f"[Epoch {epoch+1:03d}/{max_epochs}] "
            f"val_loss={val_loss:.4f}  mIoU={mean_IoU:.4f}  "
            f"GPU_MB={gpu_mb}"
        )
        logger.info(f"  per-class IoU: {np.round(IoU_array, 4).tolist()}")

        metrics_log.append((epoch + 1, val_loss, mean_IoU, gpu_mb))

        if mean_IoU > best_mIoU:
            best_mIoU = mean_IoU
            best_path = os.path.join(output_dir, "best.pt")
            torch.save(model.module.state_dict(), best_path)
            logger.info(f"  *** New best mIoU={best_mIoU:.4f} -> saved {best_path}")
            # update checkpoint with new best_mIoU
            torch.save(
                {
                    "epoch": epoch + 1,
                    "best_mIoU": best_mIoU,
                    "state_dict": model.module.state_dict(),
                    "optimizer": optimizer.state_dict(),
                },
                ckpt_path,
            )

    # ── summary ───────────────────────────────────────────────────────────────
    elapsed = timeit.default_timer() - t0
    logger.info("=" * 60)
    logger.info(f"Training complete  ({elapsed/60:.1f} min)")
    logger.info(f"{'Epoch':>6}  {'val_loss':>9}  {'mIoU':>7}  {'GPU_MB':>8}")
    for ep, vl, mi, gm in metrics_log:
        logger.info(f"{ep:>6}  {vl:>9.4f}  {mi:>7.4f}  {gm:>8.1f}")
    logger.info(f"Best mIoU : {best_mIoU:.4f}")
    logger.info(f"Checkpoint: {ckpt_path}")

    writer_dict["writer"].close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--epochs", type=int, default=5,
                        help="Number of epochs to train (default 5)")
    args = parser.parse_args()
    main(max_epochs=args.epochs)
