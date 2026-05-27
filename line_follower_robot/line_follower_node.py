#!/usr/bin/env python3
"""
line_follower_node.py — based on original working 2-wheel version
Key changes from original:
  - rgb8 encoding (camera sends rgb8 not bgr8)
  - Never permanently stops — keeps rotating until line found
  - Stronger rotation speed on recovery (0.4 rad/s fixed, not kp*error)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
try:
    from cv_bridge import CvBridge
except ImportError:
    raise SystemExit("cv_bridge required: sudo apt install ros-jazzy-cv-bridge")

# ── Tuning (same as original working version) ─────────────────────────────────
BASE_SPEED    = 0.12
MAX_SPEED     = 0.18
SPEED_DECAY   = 0.004
KP            = 0.005
BLACK_THRESH  = 80
SCAN_ROW_FRAC = 0.65   # scan row at 65% down — adjusted for forward-down camera
RECOVERY_SPEED = 0.40  # rad/s — strong enough to sweep through a corner
KD            = 0.0    # derivative gain (can be set from launch)
# ─────────────────────────────────────────────────────────────────────────────


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__("line_follower_node")

        self.declare_parameter("base_speed",    BASE_SPEED)
        self.declare_parameter("kp",            KP)
        self.declare_parameter("kd",            KD)
        self.declare_parameter("black_thresh",  BLACK_THRESH)
        self.declare_parameter("debug_image",   True)

        self.base_speed   = self.get_parameter("base_speed").value
        self.kp           = self.get_parameter("kp").value
        self.kd           = self.get_parameter("kd").value
        self.black_thresh = self.get_parameter("black_thresh").value
        self.debug        = self.get_parameter("debug_image").value

        self.bridge     = CvBridge()
        self.last_error = 0.0
        self.lost_count = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub = self.create_subscription(
            Image, "/line_camera/image", self.image_callback, sensor_qos)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        if self.debug:
            self.debug_pub = self.create_publisher(Image, "/line_camera/debug", 10)

        self.get_logger().info(
            f"Line Follower started | KP={self.kp} | "
            f"base_speed={self.base_speed} | thresh={self.black_thresh}"
        )

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge: {e}")
            return

        h, w = frame.shape[:2]

        # Grayscale + threshold (black line → white pixels in mask)
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        _, mask = cv2.threshold(gray, self.black_thresh, 255, cv2.THRESH_BINARY_INV)

        # Single scan row at 75% — same as original
        scan_y   = int(h * SCAN_ROW_FRAC)
        scan_row = mask[scan_y, :]
        indices  = np.where(scan_row > 0)[0]

        cmd = Twist()

        if len(indices) > 0:
            centroid_x      = float(np.mean(indices))
            self.lost_count = 0
            error           = centroid_x - (w / 2.0)
            # PD control: proportional on pixel error, derivative on pixel delta
            delta_error     = error - self.last_error
            cmd.angular.z = -(self.kp * error + self.kd * delta_error)
            self.last_error = error
            cmd.linear.x  = float(np.clip(
                self.base_speed - SPEED_DECAY * abs(error), 0.05, MAX_SPEED))

            self.get_logger().info(
                f"FOLLOW  err={error:+.1f}  "
                f"vx={cmd.linear.x:.2f}  wz={cmd.angular.z:+.3f}",
                throttle_duration_sec=0.5)
        else:
            # Line lost — rotate toward last known direction, NEVER stop
            self.lost_count += 1
            direction        = -1.0 if self.last_error > 0 else 1.0
            cmd.linear.x     = 0.0
            cmd.angular.z    = direction * RECOVERY_SPEED

            if self.lost_count % 15 == 0:
                side = "RIGHT" if direction < 0 else "LEFT"
                self.get_logger().warn(
                    f"LOST {self.lost_count} frames — rotating {side} "
                    f"at {RECOVERY_SPEED} rad/s"
                )

        self.cmd_pub.publish(cmd)

        # Debug image
        if self.debug:
            dbg      = frame.copy()
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            dbg      = cv2.addWeighted(dbg, 0.7, mask_bgr, 0.3, 0)
            cv2.line(dbg, (0, scan_y), (w-1, scan_y), (0, 255, 0), 1)
            if len(indices) > 0:
                cx = int(centroid_x)   # type: ignore
                cv2.circle(dbg, (cx, scan_y), 5, (0, 0, 255), -1)
                cv2.line(dbg, (w//2, scan_y), (cx, scan_y), (255, 0, 0), 2)
                cv2.putText(dbg, f"err:{error:.1f}", (4, h-6),   # type: ignore
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)
            else:
                cv2.putText(dbg, f"LOST:{self.lost_count}", (4, h-6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
            try:
                dm = self.bridge.cv2_to_imgmsg(dbg, encoding="rgb8")
                dm.header = msg.header
                self.debug_pub.publish(dm)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
