#!/usr/bin/env python3
"""
yolo_depth_probe_tf.py (ROS 2 Jazzy) - Roboflow Serverless Model

Runs Roboflow (InferenceHTTPClient) object detection on color frames,
samples depth at detection center, transforms point into base frame using TF.

Robust features:
  - Approx time sync (color + depth + camera_info [+ optional depth_info])
  - Depth median window + multi-offset fallback
  - TF lookup using message timestamp (fallback to slightly older stamp, then latest)
  - EMA smoothing on BASE xyz
  - Outlier rejection (prevents sudden teleport jumps)
  - Debug GUI overlay (cv2.imshow)
  - Safe shutdown (prevents rcl_shutdown already called)

Roboflow notes:
  - Set ROBOFLOW_API_KEY in environment instead of hardcoding.
  - Serverless calls are network requests => keep process_hz modest (e.g. 2-10).

Example (aligned depth):
  export ROBOFLOW_API_KEY="..."
  python3 yolo_depth_probe_tf.py --ros-args \
    -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    -p camera_info_topic:=/camera/camera/color/camera_info \
    -p cam_frame:=camera_color_optical_frame \
    -p base_frame:=base \
    -p debug_view:=true
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict, Any

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

from cv_bridge import CvBridge

# message_filters (sync)
from message_filters import Subscriber, ApproximateTimeSynchronizer

# Roboflow Inference SDK
from inference_sdk import InferenceHTTPClient


def _wall_time() -> float:
    return time.time()


def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


def intrinsics_from_camera_info(msg: CameraInfo) -> Intrinsics:
    fx = float(msg.k[0])
    fy = float(msg.k[4])
    cx = float(msg.k[2])
    cy = float(msg.k[5])
    return Intrinsics(fx=fx, fy=fy, cx=cx, cy=cy, width=int(msg.width), height=int(msg.height))


def depth_to_meters(depth_np: np.ndarray, depth_scale_m: float) -> np.ndarray:
    if depth_np.dtype == np.uint16:
        return depth_np.astype(np.float32) * float(depth_scale_m)
    if depth_np.dtype in (np.float32, np.float64):
        return depth_np.astype(np.float32)
    return depth_np.astype(np.float32)


def sample_depth_median(depth_m: np.ndarray, u: int, v: int, window: int, max_depth_m: float) -> Optional[float]:
    h, w = depth_m.shape[:2]
    half = max(0, window // 2)

    u0 = clamp(u - half, 0, w - 1)
    u1 = clamp(u + half, 0, w - 1)
    v0 = clamp(v - half, 0, h - 1)
    v1 = clamp(v + half, 0, h - 1)

    patch = depth_m[v0:v1 + 1, u0:u1 + 1].reshape(-1)
    patch = patch[np.isfinite(patch)]
    patch = patch[patch > 1e-6]
    patch = patch[patch < max_depth_m]
    if patch.size == 0:
        return None
    return float(np.median(patch))


def pixel_to_camera_xyz(u: float, v: float, z: float, K: Intrinsics) -> Tuple[float, float, float]:
    x = (u - K.cx) * z / K.fx
    y = (v - K.cy) * z / K.fy
    return float(x), float(y), float(z)


def transform_point(T: TransformStamped, xyz: Tuple[float, float, float]) -> Tuple[float, float, float]:
    x, y, z = xyz

    tx = float(T.transform.translation.x)
    ty = float(T.transform.translation.y)
    tz = float(T.transform.translation.z)

    qx = float(T.transform.rotation.x)
    qy = float(T.transform.rotation.y)
    qz = float(T.transform.rotation.z)
    qw = float(T.transform.rotation.w)

    # Quaternion -> rotation matrix
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz

    R = np.array([
        [1.0 - 2.0*(yy + zz),       2.0*(xy - wz),       2.0*(xz + wy)],
        [      2.0*(xy + wz), 1.0 - 2.0*(xx + zz),       2.0*(yz - wx)],
        [      2.0*(xz - wy),       2.0*(yz + wx), 1.0 - 2.0*(xx + yy)],
    ], dtype=np.float64)

    p = np.array([x, y, z], dtype=np.float64)
    p2 = R @ p + np.array([tx, ty, tz], dtype=np.float64)
    return float(p2[0]), float(p2[1]), float(p2[2])


class YoloDepthProbeTF(Node):
    def __init__(self):
        super().__init__("yolo_depth_probe_tf")

        # ---------------- Params ----------------
        # Roboflow
        self.declare_parameter("api_url", "https://serverless.roboflow.com")
        self.declare_parameter("api_key", "")  # prefer env var ROBOFLOW_API_KEY
        self.declare_parameter("model_id", "block-detection-writ8/3")
        self.declare_parameter("conf", 0.35)
        self.declare_parameter("process_hz", 10.0)
        self.declare_parameter("target_class", "")  # optional filter; empty => accept best overall

        # Frames
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("cam_frame", "camera_color_optical_frame")

        # Topics
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("depth_info_topic", "")  # optional

        # Depth conversion + filtering
        self.declare_parameter("depth_scale_m", 0.001)  # uint16 mm -> meters
        self.declare_parameter("depth_window", 9)       # median window (odd)
        self.declare_parameter("max_depth_m", 5.0)

        # Smoothing + outlier rejection
        self.declare_parameter("ema_alpha", 0.35)       # 0..1
        self.declare_parameter("max_jump_m", 0.50)      # reject base jumps bigger than this

        # GUI
        self.declare_parameter("debug_view", False)

        # Read params
        self.api_url = str(self.get_parameter("api_url").value)
        self.model_id = str(self.get_parameter("model_id").value)
        self.conf = float(self.get_parameter("conf").value)
        self.process_hz = float(self.get_parameter("process_hz").value)
        self.target_class = str(self.get_parameter("target_class").value).strip()

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.cam_frame = str(self.get_parameter("cam_frame").value)

        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.depth_info_topic = str(self.get_parameter("depth_info_topic").value).strip()

        self.depth_scale_m = float(self.get_parameter("depth_scale_m").value)
        self.depth_window = int(self.get_parameter("depth_window").value)
        if self.depth_window < 1:
            self.depth_window = 1
        if self.depth_window % 2 == 0:
            self.depth_window += 1

        self.max_depth_m = float(self.get_parameter("max_depth_m").value)

        self.ema_alpha = float(self.get_parameter("ema_alpha").value)
        self.ema_alpha = max(0.0, min(1.0, self.ema_alpha))

        self.max_jump_m = float(self.get_parameter("max_jump_m").value)
        self.max_jump_m = max(0.05, self.max_jump_m)

        self.debug_view = bool(self.get_parameter("debug_view").value)

        # Roboflow API key: param overrides env if provided
        param_key = str(self.get_parameter("api_key").value).strip()
        env_key = os.environ.get("ROBOFLOW_API_KEY", "").strip()
        self.api_key = param_key or env_key
        if not self.api_key:
            self.get_logger().warn("No Roboflow API key found. Set ROBOFLOW_API_KEY or pass -p api_key:=...")

        # ---------------- TF ----------------
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- Roboflow Client ----------------
        self.get_logger().info(f"Roboflow model_id: {self.model_id}")
        self.get_logger().info(f"Roboflow api_url:  {self.api_url}")
        self.client = InferenceHTTPClient(api_url=self.api_url, api_key=self.api_key)

        self.bridge = CvBridge()

        # State
        self._ema_base_xyz: Optional[np.ndarray] = None
        self._last_base_xyz: Optional[np.ndarray] = None
        self._last_no_det_log = 0.0
        self._last_process_time = 0.0

        # ---------------- Sync Subscribers ----------------
        self.sub_color = Subscriber(self, Image, self.color_topic)
        self.sub_depth = Subscriber(self, Image, self.depth_topic)
        self.sub_info = Subscriber(self, CameraInfo, self.camera_info_topic)
        subs = [self.sub_color, self.sub_depth, self.sub_info]

        if self.depth_info_topic:
            self.sub_depth_info = Subscriber(self, CameraInfo, self.depth_info_topic)
            subs.append(self.sub_depth_info)
        else:
            self.sub_depth_info = None

        self.sync = ApproximateTimeSynchronizer(
            fs=subs,
            queue_size=10,
            slop=0.08,
            allow_headerless=False
        )
        self.sync.registerCallback(self.on_synced)

        self.get_logger().info(
            "YOLO+Depth+TF (Roboflow) started.\n"
            f"  base_frame='{self.base_frame}' cam_frame='{self.cam_frame}'\n"
            f"  color_topic='{self.color_topic}'\n"
            f"  depth_topic='{self.depth_topic}'\n"
            f"  camera_info_topic='{self.camera_info_topic}'\n"
            f"  depth_info_topic='{self.depth_info_topic or '(none)'}'\n"
            f"  process_hz={self.process_hz} conf={self.conf} depth_window={self.depth_window} "
            f"ema_alpha={self.ema_alpha} max_jump_m={self.max_jump_m} debug_view={self.debug_view}\n"
            f"  model_id='{self.model_id}' target_class='{self.target_class or '(any)'}'"
        )

    def _log_no_dets(self):
        t = _wall_time()
        if (t - self._last_no_det_log) > 2.0:
            self.get_logger().info("No detections")
            self._last_no_det_log = t

    def _ema_update(self, base_xyz: Tuple[float, float, float]) -> Tuple[float, float, float]:
        v = np.array(base_xyz, dtype=np.float32)
        if self._ema_base_xyz is None:
            self._ema_base_xyz = v
        else:
            a = self.ema_alpha
            self._ema_base_xyz = a * v + (1.0 - a) * self._ema_base_xyz
        return float(self._ema_base_xyz[0]), float(self._ema_base_xyz[1]), float(self._ema_base_xyz[2])

    def _try_depth_samples(self, depth_m: np.ndarray, K: Intrinsics, u: int, v: int) -> Optional[Tuple[float, float, float, int, int]]:
        offsets: List[Tuple[int, int]] = [
            (0, 0),
            (-5, 0), (5, 0), (0, -5), (0, 5),
            (-10, 0), (10, 0), (0, -10), (0, 10),
            (-5, -5), (-5, 5), (5, -5), (5, 5),
        ]
        best = None  # (score, x,y,z,uu,vv)
        for du, dv in offsets:
            uu = clamp(u + du, 0, K.width - 1)
            vv = clamp(v + dv, 0, K.height - 1)
            z = sample_depth_median(depth_m, uu, vv, self.depth_window, self.max_depth_m)
            if z is None:
                continue
            x, y, zc = pixel_to_camera_xyz(uu, vv, z, K)
            score = abs(du) + abs(dv)
            if best is None or score < best[0]:
                best = (score, x, y, zc, uu, vv)
        if best is None:
            return None
        _, x, y, zc, uu, vv = best
        return (x, y, zc, uu, vv)

    def _lookup_tf(self, stamp: Time) -> Optional[TransformStamped]:
        timeout = Duration(seconds=0.2)

        # 1) exact stamp
        try:
            if self.tf_buffer.can_transform(self.base_frame, self.cam_frame, stamp, timeout=timeout):
                return self.tf_buffer.lookup_transform(self.base_frame, self.cam_frame, stamp)
        except Exception:
            pass

        # 2) slightly older (avoid “future extrapolation” by a hair)
        try:
            older = stamp - Duration(nanoseconds=int(50e6))  # 50ms
            if self.tf_buffer.can_transform(self.base_frame, self.cam_frame, older, timeout=timeout):
                return self.tf_buffer.lookup_transform(self.base_frame, self.cam_frame, older)
        except Exception:
            pass

        # 3) latest
        try:
            if self.tf_buffer.can_transform(self.base_frame, self.cam_frame, Time(), timeout=timeout):
                return self.tf_buffer.lookup_transform(self.base_frame, self.cam_frame, Time())
        except Exception:
            pass

        return None

    def _reject_jump(self, base_xyz: Tuple[float, float, float]) -> bool:
        v = np.array(base_xyz, dtype=np.float32)
        if self._last_base_xyz is None:
            return False
        dist = float(np.linalg.norm(v - self._last_base_xyz))
        return dist > self.max_jump_m

    def _parse_roboflow_predictions(self, result: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Roboflow serverless typically returns:
          {"predictions":[{"x":..,"y":..,"width":..,"height":..,"class":"...","confidence":..}, ...], ...}

        Some modes may return different keys; we guard for that.
        """
        preds = result.get("predictions", None)
        if isinstance(preds, list):
            return preds
        # fallback: sometimes "detections"
        preds = result.get("detections", None)
        if isinstance(preds, list):
            return preds
        return []

    def on_synced(self, *msgs):
        """
        msgs:
          if depth_info_topic provided: (color, depth, cam_info, depth_info)
          else:                        (color, depth, cam_info)
        """
        if self.process_hz > 0:
            now = _wall_time()
            min_dt = 1.0 / self.process_hz
            if (now - self._last_process_time) < min_dt:
                return
            self._last_process_time = now

        if len(msgs) == 3:
            color_msg, depth_msg, info_msg = msgs
        else:
            color_msg, depth_msg, info_msg, _depth_info_msg = msgs

        # Use color intrinsics (aligned depth uses color geometry)
        K = intrinsics_from_camera_info(info_msg)

        # Convert images
        try:
            bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge color failed: {e}")
            return

        try:
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge depth failed: {e}")
            return

        depth_np = np.asarray(depth_raw).squeeze()
        if depth_np.ndim != 2:
            self.get_logger().warn(f"Depth shape unexpected: {depth_np.shape}")
            return

        depth_m = depth_to_meters(depth_np, self.depth_scale_m)

        # Roboflow inference expects RGB (usually), but accepts np.ndarray.
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        try:
            result = self.client.infer(rgb, model_id=self.model_id)
        except Exception as e:
            self.get_logger().warn(f"Roboflow infer failed: {e}")
            return

        preds = self._parse_roboflow_predictions(result)
        if not preds:
            self._log_no_dets()
            return

        # Pick best prediction (highest confidence), optionally filtering by class name
        best = None  # (conf, pred_dict)
        for p in preds:
            conf = float(p.get("confidence", 0.0))
            cls_name = str(p.get("class", "")).strip()

            if self.target_class and cls_name != self.target_class:
                continue

            if best is None or conf > best[0]:
                best = (conf, p)

        if best is None:
            self._log_no_dets()
            return

        conf, p = best
        cls_name = str(p.get("class", "")).strip() or "object"

        # Roboflow bbox format: center x,y + width,height (pixels)
        cx = float(p.get("x", 0.0))
        cy = float(p.get("y", 0.0))
        bw = float(p.get("width", 0.0))
        bh = float(p.get("height", 0.0))

        x1 = cx - 0.5 * bw
        y1 = cy - 0.5 * bh
        x2 = cx + 0.5 * bw
        y2 = cy + 0.5 * bh

        u = int(round(cx))
        v = int(round(cy))
        u = clamp(u, 0, K.width - 1)
        v = clamp(v, 0, K.height - 1)

        sample = self._try_depth_samples(depth_m, K, u, v)
        if sample is None:
            self.get_logger().info(f"{cls_name}: no valid depth")
            return

        cam_x, cam_y, cam_z, uu, vv = sample

        # TF
        stamp = Time.from_msg(color_msg.header.stamp)
        T = self._lookup_tf(stamp)
        if T is None:
            self.get_logger().warn(
                f"{cls_name}: CAM[{self.cam_frame}] xyz=({cam_x:.3f},{cam_y:.3f},{cam_z:.3f}) m | "
                f"TF missing {self.base_frame}<-{self.cam_frame}"
            )
            return

        base_xyz = transform_point(T, (cam_x, cam_y, cam_z))

        # Reject huge jumps
        if self._reject_jump(base_xyz):
            self.get_logger().warn(
                f"JUMP_REJECT {cls_name} conf={conf:.2f} | "
                f"CAM[{self.cam_frame}] xyz=({cam_x:.3f},{cam_y:.3f},{cam_z:.3f}) m | "
                f"BASE[{self.base_frame}] xyz=({base_xyz[0]:.3f},{base_xyz[1]:.3f},{base_xyz[2]:.3f})"
            )
            return

        # Update last good
        self._last_base_xyz = np.array(base_xyz, dtype=np.float32)
        base_xyz_s = self._ema_update(base_xyz)

        self.get_logger().info(
            f"{cls_name} conf={conf:.2f} | "
            f"CAM[{self.cam_frame}] xyz=({cam_x:.3f},{cam_y:.3f},{cam_z:.3f}) m (u,v)=({uu},{vv}) | "
            f"BASE[{self.base_frame}] xyz=({base_xyz[0]:.3f},{base_xyz[1]:.3f},{base_xyz[2]:.3f}) | "
            f"BASE_SMOOTH xyz=({base_xyz_s[0]:.3f},{base_xyz_s[1]:.3f},{base_xyz_s[2]:.3f})"
        )

        if self.debug_view:
            vis = bgr.copy()
            cv2.rectangle(
                vis,
                (int(max(0, x1)), int(max(0, y1))),
                (int(min(K.width - 1, x2)), int(min(K.height - 1, y2))),
                (0, 255, 0),
                2
            )
            cv2.circle(vis, (int(uu), int(vv)), 4, (0, 0, 255), -1)
            cv2.putText(
                vis,
                f"{cls_name} {conf:.2f} z={cam_z:.2f}m",
                (int(max(0, x1)), max(0, int(max(0, y1)) - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
            cv2.imshow("yolo_depth_probe_tf (roboflow)", vis)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = YoloDepthProbeTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        try:
            node.get_logger().error(f"Unhandled exception: {e}")
        except Exception:
            pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        # Safe shutdown
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
