"""
PIDNet-S RELLIS-3D test script.
Runs testval against the RELLIS test split (test.lst).
Loads best.pt from the training output directory.
Reports per-class IoU, mIoU, pixel accuracy, and inference time.
"""

import os
import sys
import logging
import timeit

import numpy as np
import torch
import torch.backends.cudnn as cudnn
from torch.nn import functional as F
from tqdm import tqdm

sys.path.insert(0, os.path.dirname(__file__))

import models
import datasets
from configs import config, update_config
from utils.utils import get_confusion_matrix

# RELLIS class names (index = training ID)
CLASS_NAMES = [
    "void",       # 0
    "grass",      # 1
    "mud",        # 2
    "bush",       # 3
    "concrete",   # 4
    "sky",        # 5
    "water",      # 6
    "puddle",     # 7
    "dirt",       # 8
    "gravel",     # 9
    "asphalt",    # 10
    "building",   # 11
    "log",        # 12
    "person",     # 13
    "fence",      # 14
    "vehicle",    # 15
    "object",     # 16
    "pole",       # 17
    "tree trunk", # 18
]


class Args:
    cfg = "configs/rellis/pidnet_small_rellis.yaml"
    # Override TEST_SET to the actual test split, not val
    opts = ["DATASET.TEST_SET", "Rellis_3D_image_split/test.lst"]


def main():
    update_config(config, Args())

    output_dir = os.path.join("output", "rellis", "pidnet_small_rellis")
    model_file = os.path.join(output_dir, "best.pt")

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(message)s",
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler(os.path.join(output_dir, "test_results.log"), mode="w"),
        ],
    )
    logger = logging.getLogger()

    logger.info("=== PIDNet-S RELLIS-3D TEST ===")
    logger.info(f"Model     : {model_file}")
    logger.info(f"Test set  : {config.DATASET.TEST_SET}")
    logger.info(f"GPU       : {torch.cuda.get_device_name(0)}")

    assert "test.lst" in config.DATASET.TEST_SET, (
        f"TEST_SET should point to test.lst, got: {config.DATASET.TEST_SET}"
    )

    cudnn.benchmark = True
    cudnn.enabled   = True

    # ── model ─────────────────────────────────────────────────────────────────
    model = models.pidnet.get_seg_model(config, imgnet_pretrained=False)

    state = torch.load(model_file, map_location="cpu")
    # best.pt is saved as model.module.state_dict() from FullModel(DataParallel)
    # keys look like "model.xxx" — strip the "model." prefix to load into bare model
    if any(k.startswith("model.") for k in state.keys()):
        state = {k[len("model."):]: v for k, v in state.items() if k.startswith("model.")}

    missing, unexpected = model.load_state_dict(state, strict=False)
    if missing:
        logger.warning(f"Missing keys  : {missing}")
    if unexpected:
        logger.warning(f"Unexpected keys: {unexpected}")

    model = model.cuda().eval()
    logger.info("Model loaded OK")

    # ── dataset ───────────────────────────────────────────────────────────────
    test_size = (config.TEST.IMAGE_SIZE[1], config.TEST.IMAGE_SIZE[0])
    test_dataset = datasets.rellis(
        root=config.DATASET.ROOT,
        list_path=config.DATASET.TEST_SET,
        num_classes=config.DATASET.NUM_CLASSES,
        multi_scale=False,
        flip=False,
        ignore_label=config.TRAIN.IGNORE_LABEL,
        base_size=config.TEST.BASE_SIZE,
        crop_size=test_size,
    )

    testloader = torch.utils.data.DataLoader(
        test_dataset,
        batch_size=1,
        shuffle=False,
        num_workers=0,
        pin_memory=False,
    )

    logger.info(f"Test samples: {len(test_dataset)}")

    # ── inference ─────────────────────────────────────────────────────────────
    confusion_matrix = np.zeros(
        (config.DATASET.NUM_CLASSES, config.DATASET.NUM_CLASSES)
    )

    inference_times = []
    t_total_start = timeit.default_timer()

    with torch.no_grad():
        for idx, batch in enumerate(tqdm(testloader, desc="Testing")):
            image, label, _, _, name = batch
            size = label.size()
            image = image.cuda()

            t0 = timeit.default_timer()

            pred = model(image)
            # NUM_OUTPUTS=2: model returns list; take penultimate (semantic) output
            if isinstance(pred, (list, tuple)):
                pred = pred[-2]

            pred = F.interpolate(
                pred, size=size[-2:],
                mode="bilinear", align_corners=config.MODEL.ALIGN_CORNERS,
            )

            torch.cuda.synchronize()
            inference_times.append(timeit.default_timer() - t0)

            confusion_matrix += get_confusion_matrix(
                label, pred, size,
                config.DATASET.NUM_CLASSES,
                config.TRAIN.IGNORE_LABEL,
            )

    t_total = timeit.default_timer() - t_total_start

    # ── metrics ───────────────────────────────────────────────────────────────
    pos        = confusion_matrix.sum(1)
    res        = confusion_matrix.sum(0)
    tp         = np.diag(confusion_matrix)
    pixel_acc  = tp.sum() / pos.sum()
    mean_acc   = (tp / np.maximum(1.0, pos)).mean()
    IoU_array  = tp / np.maximum(1.0, pos + res - tp)
    mean_IoU   = IoU_array.mean()

    avg_ms  = np.mean(inference_times) * 1000
    med_ms  = np.median(inference_times) * 1000
    fps     = 1.0 / np.mean(inference_times)

    logger.info("")
    logger.info("=" * 60)
    logger.info("RESULTS")
    logger.info("=" * 60)
    logger.info(f"mIoU         : {mean_IoU:.4f}")
    logger.info(f"Pixel Acc    : {pixel_acc:.4f}")
    logger.info(f"Mean Acc     : {mean_acc:.4f}")
    logger.info(f"Avg latency  : {avg_ms:.1f} ms  (median {med_ms:.1f} ms)  {fps:.1f} FPS")
    logger.info(f"Total time   : {t_total:.1f} s  over {len(test_dataset)} images")
    logger.info("")
    logger.info(f"{'ID':>3}  {'Class':<12}  {'IoU':>7}  {'Acc':>7}")
    logger.info("-" * 38)
    for i, (iou, acc) in enumerate(zip(IoU_array, tp / np.maximum(1.0, pos))):
        name_str = CLASS_NAMES[i] if i < len(CLASS_NAMES) else f"cls{i}"
        logger.info(f"{i:>3}  {name_str:<12}  {iou:>7.4f}  {acc:>7.4f}")
    logger.info("-" * 38)
    logger.info(f"{'':>3}  {'MEAN':<12}  {mean_IoU:>7.4f}  {mean_acc:>7.4f}")
    logger.info("=" * 60)
    logger.info(f"Results saved -> {output_dir}/test_results.log")


if __name__ == "__main__":
    main()
