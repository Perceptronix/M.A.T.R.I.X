# PIDNet Data

This directory contains the local Rellis-3D image-segmentation data layout used by this checkout of PIDNet. The dataset pairs synchronized RGB camera frames with single-channel semantic label-ID masks and provides train, validation, and test manifests.

## Scope and status

The files in this directory are a prepared local copy, not a downloader or a complete dataset-preparation pipeline. The repository contains the data and split lists, but it does not currently contain a Rellis-specific dataset class or a checked-in label-ID-to-training-class mapping. Any training or evaluation integration must therefore define how the Rellis IDs are converted to the model's contiguous class IDs.

The manifest lists are the most reliable definition of the usable subset. They reference paths relative to `PIDNet/data` and contain one image path followed by its matching label path on each line.

## Directory layout

```text
data/
|-- README.md
|-- Rellis-3D/
|   |-- 00000/
|   |   |-- pylon_camera_node/             # RGB camera frames (.jpg)
|   |   |-- pylon_camera_node_label_id/    # semantic masks (.png)
|   |-- 00001/
|   |-- 00002/
|   |-- 00003/
|   |-- 00004/
|-- Rellis_3D_image_split/
|   |-- train.lst
|   |-- val.lst
|   |-- test.lst
|-- Rellis_3D_pylon_camera_node_label_id/
    |-- Rellis-3D/                         # mirrored label-oriented copy
```

The canonical tree is `Rellis-3D`. Each frame is identified by a filename such as:

```text
frame000000-1581624652_750.jpg
```

The matching label keeps the same basename and changes the extension and directory:

```text
Rellis-3D/00000/pylon_camera_node/frame000000-1581624652_750.jpg
Rellis-3D/00000/pylon_camera_node_label_id/frame000000-1581624652_750.png
```

The timestamp-like suffix is part of the source filename. It should be treated as an identifier, not parsed as a guaranteed wall-clock timestamp without consulting the original Rellis-3D metadata.

## Dataset inventory

The following counts were verified from the manifests and local files in this checkout.

| Split | Entries | Sequences represented | Entries by sequence |
|---|---:|---|---|
| Train | 3,302 | 00000, 00002, 00003, 00004 | 00000: 699; 00002: 804; 00003: 900; 00004: 899 |
| Validation | 983 | 00000, 00001 | 00000: 346; 00001: 637 |
| Test | 1,672 | 00000, 00001, 00002 | 00000: 154; 00001: 524; 00002: 994 |
| **Total** | **5,957** | **00000-00004** | **5,957 unique image-label pairs** |

The five canonical sequence directories contain approximately 13,556 camera JPEGs and 6,234 PNG masks. The manifests intentionally select only 5,957 pairs. Do not infer the training set by recursively globbing every image; use the relevant `.lst` file so that the intended split is preserved.

## Manifest format

Every non-empty line has exactly two whitespace-separated, repository-relative paths:

```text
<image-relative-path> <label-relative-path>
```

Example:

```text
00000/pylon_camera_node/frame000308-1581624683_550.jpg 00000/pylon_camera_node_label_id/frame000308-1581624683_550.png
```

To resolve a line from the repository root, prepend `PIDNet/data/` to both paths. To resolve it from inside this directory, prepend only `./` or use the path directly. A loader should verify both files exist and should match the image and label basenames before decoding them.

The checked-in manifests have no blank entries, no duplicate image paths, and no missing referenced files. They are disjoint across train, validation, and test in this checkout.

## File formats and semantics

### Camera images

- Extension: `.jpg`
- Source directory: `Rellis-3D/<sequence>/pylon_camera_node/`
- Verified example shape: `1200 x 1920 x 3`
- Verified decoded type: 8-bit, three-channel image
- When decoded with OpenCV, the array is BGR until converted to RGB or processed using the repository's expected convention.

### Label masks

- Extension: `.png`
- Source directory: `Rellis-3D/<sequence>/pylon_camera_node_label_id/`
- Verified example shape: `1200 x 1920`
- Verified decoded type: single-channel `uint8`
- Labels are stored as integer IDs, not as display-ready RGB colors.
- A representative mask contains IDs `3, 4, 7, 8, 9, 18, 19, 31, 33`; this sample is not a complete class taxonomy.
- Preserve nearest-neighbor interpolation for masks. Bilinear or bicubic resizing creates invalid intermediate class IDs.
- Do not normalize masks like images. Read them as integer arrays and map IDs explicitly.

The current PIDNet base dataset code uses `ignore_label=255` by default and creates a boundary target from the label mask. A Rellis adapter should decide whether `255` is the correct ignore value after the Rellis label mapping is defined, and should ensure that valid class IDs are contiguous in the range expected by the loss and model configuration.

## Recommended loading workflow

1. Select one manifest: `Rellis_3D_image_split/train.lst`, `val.lst`, or `test.lst`.
2. Read each line as an image path and a label path.
3. Resolve both paths under this `data` directory.
4. Load the image as an 8-bit color image and the mask as an unchanged single-channel array.
5. Assert that image and mask dimensions match.
6. Apply the Rellis label-ID mapping before loss calculation.
7. Resize images with a continuous interpolation method and masks with nearest-neighbor interpolation.
8. Apply the model's normalization only to the image. Keep labels integer-valued.
9. Use the training manifest only for fitting and keep validation/test manifests isolated for measurement.

A minimal path-resolution example from the `PIDNet` project root is:

```python
from pathlib import Path

DATA_ROOT = Path("data")
manifest = DATA_ROOT / "Rellis_3D_image_split" / "train.lst"

with manifest.open(encoding="utf-8") as split_file:
    for line in split_file:
        image_name, label_name = line.split()
        image_path = DATA_ROOT / image_name
        label_path = DATA_ROOT / label_name
        assert image_path.is_file(), image_path
        assert label_path.is_file(), label_path
```

This snippet only resolves files. It does not perform label conversion, augmentation, normalization, or class-count configuration.

## Important repository details

### Mirrored label-oriented tree

`Rellis_3D_pylon_camera_node_label_id/Rellis-3D/` contains a second copy of the label directories and masks, with the same sequence organization. It is not referenced by the checked-in manifests. Treat `Rellis-3D/` as the canonical source unless a data-preparation script explicitly requires the mirrored tree. Avoid combining both trees in a recursive dataset scan, which would double-count labels.

### Stray JPEG in label directories

There is one `.jpg` named `frame001493-1581624802_049.jpg` inside the `00000/pylon_camera_node_label_id` directory in both trees. It is not part of the manifest pairs and should not be treated as a label. Dataset loaders should filter by the expected extension and directory role, or preferably consume the manifests directly.

### Missing project integration

The generic `PIDNet/datasets/base_dataset.py` provides image normalization, resizing, cropping, flipping, and boundary-target generation, but this checkout does not provide the Rellis-specific subclass, class names, class count, or label conversion table. Those pieces must be added before using these files in a standard PIDNet training command.

## Data-quality checks

Before training, verify:

- Every manifest row resolves to two existing files.
- Image and mask basenames match after changing `.jpg` to `.png`.
- Image and mask dimensions match.
- Masks are single-channel integer arrays.
- The observed mask IDs are documented and mapped explicitly.
- No split overlap is introduced by custom list generation.
- The loader reads `Rellis-3D` and does not accidentally include the mirrored tree.

A quick shell-level count from the `PIDNet` project root is:

```powershell
Get-Content data/Rellis_3D_image_split/train.lst | Measure-Object -Line
Get-Content data/Rellis_3D_image_split/val.lst | Measure-Object -Line
Get-Content data/Rellis_3D_image_split/test.lst | Measure-Object -Line
```

These commands count manifest lines; they do not validate file existence or label contents.

## Reproducibility notes

- Keep the `.lst` files under version control when changing a split.
- Record the label-ID mapping and resulting number of classes next to the model configuration.
- Record whether the mirrored tree was used or ignored.
- Preserve the original mask IDs until the explicit mapping step; renaming files or converting masks to color images loses information needed for reproducible training.
- When publishing results, report the exact manifest, class mapping, ignore label, image resize/crop policy, and augmentation settings.

## External dataset reference

For dataset provenance, sensor details, original annotations, and the authoritative semantic taxonomy, consult the original Rellis-3D dataset release and its documentation. This README describes only the files and conventions verifiable in the local `PIDNet/data` directory.
