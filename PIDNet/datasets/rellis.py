import os
import cv2
import numpy as np
from torch.utils import data

from .base_dataset import BaseDataset


class Rellis(BaseDataset):
    """
    RELLIS-3D semantic segmentation dataset for PIDNet.

    Manifest format:
        image_path label_path

    Paths are relative to the dataset root.
    """

    # Official RELLIS-3D raw label ID -> PIDNet training ID.
    ID_MAPPING = {
        0: 0,
        1: 0,
        3: 1,
        4: 2,
        5: 3,
        6: 4,
        7: 5,
        8: 6,
        9: 7,
        10: 8,
        12: 9,
        15: 10,
        17: 11,
        18: 12,
        19: 13,
        23: 14,
        27: 15,
        29: 1,
        30: 1,
        31: 16,
        32: 4,
        33: 17,
        34: 18,
    }

    NUM_CLASSES = 19

    def __init__(
        self,
        root,
        list_path,
        num_classes=19,
        multi_scale=True,
        flip=True,
        ignore_label=-1,
        base_size=2048,
        crop_size=(512, 1024),
        scale_factor=16,
    ):
        super().__init__(
            ignore_label=ignore_label,
            base_size=base_size,
            crop_size=crop_size,
            scale_factor=scale_factor,
        )

        self.root = root
        self.list_path = list_path
        self.num_classes = num_classes
        self.multi_scale = multi_scale
        self.flip = flip

        self.files = []

        manifest = os.path.join(root, list_path)

        with open(manifest, "r") as f:
            for line in f:
                line = line.strip()

                if not line:
                    continue

                image_path, label_path = line.split()

                self.files.append({
                    "img": os.path.join(root, "Rellis-3D", image_path),
                    "label": os.path.join(root, "Rellis-3D", label_path),
                    "name": os.path.splitext(
                        os.path.basename(image_path)
                    )[0],
                })

        self.class_weights = np.ones(self.num_classes, dtype=np.float32)

    def __len__(self):
        return len(self.files)

    def _convert_label(self, label):
        converted = np.full(
            label.shape,
            self.ignore_label,
            dtype=np.int64
        )

        for raw_id, train_id in self.ID_MAPPING.items():
            converted[label == raw_id] = train_id

        return converted

    def __getitem__(self, index):
        item = self.files[index]

        image = cv2.imread(item["img"], cv2.IMREAD_COLOR)
        label = cv2.imread(item["label"], cv2.IMREAD_GRAYSCALE)

        if image is None:
            raise FileNotFoundError(
                f"Could not read image: {item['img']}"
            )

        if label is None:
            raise FileNotFoundError(
                f"Could not read label: {item['label']}"
            )

        if image.shape[:2] != label.shape[:2]:
            raise ValueError(
                f"Image/label size mismatch: "
                f"{image.shape[:2]} vs {label.shape[:2]}"
            )

        label = self._convert_label(label)

        size = image.shape[:2]

        if self.multi_scale or self.flip:
            image, label, edge = self.gen_sample(
                image,
                label,
                multi_scale=self.multi_scale,
                is_flip=self.flip,
            )
        else:
            image = self.input_transform(image)
            image = image.transpose((2, 0, 1))
            label = label.astype(np.int64)

            edge = cv2.Canny(
                label.astype(np.uint8),
                0,
                1
            )
            edge = (edge > 0).astype(np.float32)

        return image.copy(), label.copy(), edge.copy(), np.array(size), item["name"]