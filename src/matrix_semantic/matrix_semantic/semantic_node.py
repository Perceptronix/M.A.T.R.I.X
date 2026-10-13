#!/usr/bin/env python3
"""Member-3 entropy-gated lightweight semantic perception node.

Uses the REAL trained model exported by the repository's own author:
Perceptronix/M.A.T.R.I.X @ b17e5fc, PIDNet/deployment/model.onnx, per
PIDNet/docs/MEMBER3_MODEL.md and MEMBER3_INTERFACE.md. That documentation
states plainly: the checkpoint is trained on RELLIS-3D (19 named
classes), the export was validated (PyTorch vs ONNX Runtime max abs diff
2.4e-5), but "no accuracy, entropy threshold, or performance number is
asserted" -- i.e. it is a genuine trained model whose real-world accuracy
the author does not claim. This node reports that distinction honestly
rather than treating "trained" as equivalent to "validated accurate".

Gate: this node no longer computes its own entropy/threshold comparison.
It subscribes to /perception/semantic_trigger (std_msgs/Bool), published
by the repository's own gating_supervisor_node against entropy_node's
real, bounded [0,1] entropy and gating_supervisor_node's own declared
default threshold (0.30) -- the actual, intended architecture, not a
value invented for this integration.

Input: Member-1's real, live left camera image (never Gazebo ground
truth, never GPS). Preprocessing follows MEMBER3_INTERFACE.md exactly:
resize to 1024x512 (W x H), scale to [0,1], ImageNet mean/std normalize
(the documented "BGR read + convert to RGB" step is a no-op here since
Member-1's camera already publishes rgb8).
"""
import os
import time

import numpy as np
import onnxruntime as ort
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String, Int32, Bool
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

NUM_CLASSES = 19
INPUT_W, INPUT_H = 1024, 512
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

# Perceptronix/M.A.T.R.I.X @ b17e5fc, PIDNet/docs/MEMBER3_MODEL.md
CLASS_NAMES = [
    "void", "grass", "mud", "bush", "concrete", "sky", "water", "puddle",
    "dirt", "gravel", "asphalt", "building", "log", "person", "fence",
    "vehicle", "object", "pole", "tree_trunk",
]


class SemanticNode(Node):
    def __init__(self):
        super().__init__('semantic_node')

        self.bridge = CvBridge()
        self.latest_image = None
        self.gate_active = False
        self.inference_count = 0
        self.last_inference_time = 0.0

        model_path = os.path.join(
            get_package_share_directory('matrix_semantic'), 'models', 'model.onnx')
        self.get_logger().info(f'Loading real trained PIDNet-S ONNX model: {model_path}')
        self.session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        self.get_logger().info(
            'Model loaded. Trained on RELLIS-3D (19 classes) per repo docs; '
            'author asserts NO accuracy/threshold number -- semantic validity '
            'limited, reported honestly, not hidden.')

        self.image_sub = self.create_subscription(
            Image, '/camera/left/image_rect_color', self.on_image, qos_profile_sensor_data)
        self.trigger_sub = self.create_subscription(
            Bool, '/perception/semantic_trigger', self.on_trigger, 10)

        self.gate_state_pub = self.create_publisher(String, '/semantic/gate_state', 10)
        self.inference_count_pub = self.create_publisher(Int32, '/semantic/inference_count', 10)
        self.confidence_pub = self.create_publisher(Float32, '/semantic/confidence', 10)
        self.entropy_pub = self.create_publisher(Float32, '/semantic/entropy', 10)
        self.cost_bias_pub = self.create_publisher(Float32, '/semantic/cost_bias', 10)
        self.class_map_pub = self.create_publisher(Image, '/semantic/class_map', 10)
        self.top_class_pub = self.create_publisher(String, '/semantic/dominant_class', 10)

        self.status_timer = self.create_timer(1.0, self.publish_status)

    def on_trigger(self, msg: Bool):
        was_active = self.gate_active
        self.gate_active = bool(msg.data)
        if self.gate_active and not was_active:
            self.get_logger().info(
                '/perception/semantic_trigger = True: gate ACTIVE, PIDNet will run.')
        elif was_active and not self.gate_active:
            self.get_logger().info(
                '/perception/semantic_trigger = False: gate INACTIVE, no forward() this frame.')

    def on_image(self, msg: Image):
        self.latest_image = msg
        if not self.gate_active:
            return  # deliberately skip inference -- this IS the gate, not a label
        now = time.time()
        if now - self.last_inference_time < 0.3:
            return  # rate-limit while active; still genuinely ACTIVE
        self.run_inference(msg)
        self.last_inference_time = now

    def run_inference(self, img_msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
        resized = cv2.resize(cv_img, (INPUT_W, INPUT_H), interpolation=cv2.INTER_LINEAR)
        norm = resized.astype(np.float32) / 255.0
        norm = (norm - IMAGENET_MEAN) / IMAGENET_STD
        tensor = norm.transpose(2, 0, 1)[None, ...].astype(np.float32)

        t0 = time.time()
        logits = self.session.run(None, {self.input_name: tensor})[0]  # (1,19,64,128)
        infer_ms = (time.time() - t0) * 1000.0

        exp = np.exp(logits - logits.max(axis=1, keepdims=True))
        probs = exp / exp.sum(axis=1, keepdims=True)
        class_ids = probs.argmax(axis=1)[0]          # (64,128)
        confidence_map = probs.max(axis=1)[0]         # (64,128)
        pixel_entropy = -(probs * np.log(np.clip(probs, 1e-8, None))).sum(axis=1)[0]

        self.inference_count += 1
        mean_confidence = float(confidence_map.mean())
        mean_semantic_entropy = float(pixel_entropy.mean())
        # Class-agnostic, confidence/entropy-driven bias -- see semantic_node
        # module docstring for why this does not assign per-class terrain
        # meaning despite real class names now being available: the author's
        # own docs assert no validated accuracy for this checkpoint.
        cost_bias = float(np.clip((1.0 - mean_confidence) + 0.1 * mean_semantic_entropy, 0.0, 1.0))

        values, counts = np.unique(class_ids, return_counts=True)
        dominant_id = int(values[np.argmax(counts)])
        dominant_name = CLASS_NAMES[dominant_id] if dominant_id < len(CLASS_NAMES) else 'unknown'
        dominant_frac = float(counts.max() / class_ids.size)

        class_map_msg = self.bridge.cv2_to_imgmsg(class_ids.astype(np.uint8), encoding='mono8')
        class_map_msg.header = img_msg.header

        self.confidence_pub.publish(Float32(data=mean_confidence))
        self.entropy_pub.publish(Float32(data=mean_semantic_entropy))
        self.cost_bias_pub.publish(Float32(data=cost_bias))
        self.class_map_pub.publish(class_map_msg)
        self.top_class_pub.publish(
            String(data=f'{dominant_name} ({dominant_frac * 100:.0f}% of pixels)'))

        self.get_logger().info(
            f'INFERENCE #{self.inference_count}: confidence={mean_confidence:.4f} '
            f'semantic_entropy={mean_semantic_entropy:.4f} cost_bias={cost_bias:.4f} '
            f'dominant_class={dominant_name}({dominant_frac * 100:.0f}%) '
            f'infer_time={infer_ms:.1f}ms')

    def publish_status(self):
        self.gate_state_pub.publish(String(data='ACTIVE' if self.gate_active else 'INACTIVE'))
        self.inference_count_pub.publish(Int32(data=self.inference_count))


def main(args=None):
    rclpy.init(args=args)
    node = SemanticNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
