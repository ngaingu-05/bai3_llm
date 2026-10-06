"""Camera-based object detector for UR3 pick-and-place environment."""

from __future__ import annotations

import json
import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger


class CameraDetector(Node):
    """Detect colored cubes via overhead camera and report scene state."""

    # Camera extrinsics (must match the SDF camera model pose)
    CAM_X, CAM_Y, CAM_Z = -0.28, 0.0, 0.75
    TABLE_Z = 0.27  # approximate z of cube centers on the table
    HFOV = math.pi / 2  # 90 degrees horizontal FOV
    IMG_W, IMG_H = 640, 480

    # Known zone positions (x, y) — must match scene.yaml
    ZONE_POSITIONS = {
        "zone_a": (-0.25, -0.30),
        "zone_b": (-0.25, -0.12),
        "zone_c": (-0.25, 0.28),
    }
    TEMPORARY_POSITIONS = {
        "temp_1": (-0.30, -0.20),
        "temp_2": (-0.30, -0.05),
    }
    ZONE_THRESHOLD = 0.06  # 6 cm proximity to count as "in zone"

    # HSV color ranges for each cube
    # Each entry is a list of (lower_hsv, upper_hsv) tuples
    COLOR_RANGES = {
        "red_cube": [
            (np.array([0, 80, 80]), np.array([10, 255, 255])),
            (np.array([170, 80, 80]), np.array([180, 255, 255])),
        ],
        "yellow_cube": [
            (np.array([18, 80, 80]), np.array([38, 255, 255])),
        ],
        "blue_cube": [
            (np.array([95, 80, 80]), np.array([135, 255, 255])),
        ],
        "green_cube": [
            (np.array([35, 60, 60]), np.array([90, 255, 255])),
        ],
        "purple_cube": [
            (np.array([125, 40, 40]), np.array([170, 255, 255])),
        ],
    }

    MIN_CONTOUR_AREA = 80  # minimum pixel area to consider a detection valid

    def __init__(self) -> None:
        super().__init__("camera_detector")
        self.bridge = CvBridge()
        self.latest_image: Optional[np.ndarray] = None
        self.latest_scene: Optional[dict] = None

        # ROS interfaces
        self.image_sub = self.create_subscription(
            Image, "/overhead_cam", self._image_cb, 10
        )
        self.scene_pub = self.create_publisher(String, "/scene_state", 10)
        self.detect_srv = self.create_service(
            Trigger, "/detect_objects", self._detect_cb
        )
        # Publish scene state every 2 seconds
        self.timer = self.create_timer(2.0, self._timer_cb)
        self.get_logger().info("Camera detector ready; waiting for images on /overhead_cam.")

    # ── Callbacks ──

    def _image_cb(self, msg: Image) -> None:
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")

    def _detect_cb(self, request, response):
        if self.latest_image is None:
            response.success = False
            response.message = '{"error": "No camera image received yet"}'
            return response
        scene = self._analyze_scene(self.latest_image)
        self.latest_scene = scene
        response.success = True
        response.message = json.dumps(scene)
        # Also publish
        msg = String()
        msg.data = response.message
        self.scene_pub.publish(msg)
        self.get_logger().info(f"Scene detected: zones={scene['zones']}")
        return response

    def _timer_cb(self) -> None:
        if self.latest_image is None:
            return
        scene = self._analyze_scene(self.latest_image)
        self.latest_scene = scene
        msg = String()
        msg.data = json.dumps(scene)
        self.scene_pub.publish(msg)

    # ── Detection logic ──

    def _analyze_scene(self, image: np.ndarray) -> dict:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detected_objects = {}

        for obj_name, ranges in self.COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

            # Morphological cleanup
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area < self.MIN_CONTOUR_AREA:
                continue

            M = cv2.moments(largest)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            wx, wy = self._pixel_to_world(cx, cy)
            location = self._find_location(wx, wy)
            detected_objects[obj_name] = {
                "xyz": [round(wx, 4), round(wy, 4), self.TABLE_Z],
                "position": location,
            }

        # Build zone occupancy map
        zones_state = {z: "empty" for z in self.ZONE_POSITIONS}
        temporary_state = {p: "empty" for p in self.TEMPORARY_POSITIONS}
        for obj_name, info in detected_objects.items():
            pos = info["position"]
            if pos in zones_state:
                zones_state[pos] = obj_name
            elif pos in temporary_state:
                temporary_state[pos] = obj_name

        return {
            "objects": detected_objects,
            "zones": zones_state,
            "temporary_positions": temporary_state,
        }

    def _pixel_to_world(self, u: int, v: int) -> tuple[float, float]:
        """Convert pixel (u, v) to world (x, y) using pinhole model.

        Camera at (CAM_X, CAM_Y, CAM_Z) looking down (-Z) after pitch=π/2 rotation.
        After R_y(π/2):
          image right (u+) → world -Y
          image down  (v+) → world -X
        """
        h = self.CAM_Z - self.TABLE_Z
        half_w = h * math.tan(self.HFOV / 2)  # half-width in meters
        half_h = half_w * self.IMG_H / self.IMG_W  # half-height in meters

        nu = (u - self.IMG_W / 2) / (self.IMG_W / 2)  # normalized [-1, 1]
        nv = (v - self.IMG_H / 2) / (self.IMG_H / 2)

        world_y = self.CAM_Y - nu * half_w
        world_x = self.CAM_X - nv * half_h
        return world_x, world_y

    def _find_location(self, wx: float, wy: float) -> str:
        """Classify a detection as task zone, temporary position, or table."""
        for zone_name, (zx, zy) in self.ZONE_POSITIONS.items():
            dist = math.sqrt((wx - zx) ** 2 + (wy - zy) ** 2)
            if dist < self.ZONE_THRESHOLD:
                return zone_name
        for position_name, (px, py) in self.TEMPORARY_POSITIONS.items():
            dist = math.sqrt((wx - px) ** 2 + (wy - py) ** 2)
            if dist < self.ZONE_THRESHOLD:
                return position_name
        return "table"


def main(args=None):
    rclpy.init(args=args)
    node = CameraDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
