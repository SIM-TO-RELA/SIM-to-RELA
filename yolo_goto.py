# THIS FILE DETECTS THE CUBE WITH REALSENSE + YOLO,
# CONVERTS CAMERA COORDINATE TO GENESIS COORDINATE,
# THEN TELLS THE ROBOT TO GO THERE ONCE AT Z = 0.3 AND STOPS.

import os
import threading
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

import genesis as gs
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from test_gripper import RobotiqHandE_URCapSocket


class KeyboardDevice:
    def __init__(self):
        self.pressed_keys = set()
        self.lock = threading.Lock()

    def start(self):
        pass

    def stop(self):
        pass


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class Teleop(Node):
    def __init__(self):
        super().__init__('xyz_step')
        self.pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.joints = None
        self.create_subscription(JointState, '/joint_states', self.js_cb, 10)

    def js_cb(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.joints = [name_to_pos[j] for j in JOINT_NAMES]
        except KeyError:
            pass

    def send(self):
        if self.joints is None:
            return

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = self.joints
        pt.time_from_start.sec = 1
        traj.points = [pt]

        self.pub.publish(traj)


def cam_to_genesis(x_cam, y_cam):
    x_gen = 0.9927 * x_cam - 0.0512 * y_cam - 0.4210
    y_gen = -0.0214 * x_cam - 1.0190 * y_cam - 0.0892
    return x_gen, y_gen


def detect_cube_genesis_xy():
    model = YOLO("my_model.pt")

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    align = rs.align(rs.stream.color)

    print("Detecting cube from RealSense... Press q to quit.")

    gx, gy = None, None

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

            results = model.predict(source=frame, verbose=False)
            r = results[0]

            found = False

            for box in r.boxes:
                cls_id = int(box.cls[0])
                label = model.names[cls_id]
                conf = float(box.conf[0])

                if label != "cube":
                    continue

                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

                px = int((x1 + x2) / 2)
                py = int((y1 + y2) / 2)

                depth = depth_frame.get_distance(px, py)
                if depth <= 0:
                    continue

                X, Y, Z = rs.rs2_deproject_pixel_to_point(intrinsics, [px, py], depth)
                gx, gy = cam_to_genesis(X, Y)

                print(
                    f"Camera -> X:{X:.4f}, Y:{Y:.4f}, Z:{Z:.4f} | "
                    f"Genesis -> X:{gx:.4f}, Y:{gy:.4f}, Z:0.3000 | "
                    f"conf:{conf:.2f}"
                )

                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.circle(frame, (px, py), 5, (0, 255, 0), -1)
                cv2.putText(
                    frame,
                    f"GX:{gx:.2f} GY:{gy:.2f} Z:0.30",
                    (px + 10, py - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

                cv2.imshow("Cube Detection", frame)
                cv2.waitKey(1000)

                found = True
                break

            if found:
                break

            cv2.imshow("Cube Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    return gx, gy


def build_scene():
    gs.init(precision="32", logging_level="info", backend=gs.cpu)
    np.set_printoptions(precision=7, suppress=True)

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            substeps=4,
        ),
        rigid_options=gs.options.RigidOptions(
            enable_joint_limit=True,
            enable_collision=True,
            gravity=(0, 0, -9.8),
            box_box_detection=True,
            constraint_timeconst=0.01,
        ),
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(-1.5, 0.0, 1.7),
            camera_lookat=(0.2, 0.0, 0.1),
            camera_fov=50,
            max_FPS=60,
        ),
        show_viewer=True,
        show_FPS=False,
    )

    entities = dict()

    entities["plane"] = scene.add_entity(gs.morphs.Plane())

    entities["robot"] = scene.add_entity(
        material=gs.materials.Rigid(gravity_compensation=1),
        morph=gs.morphs.MJCF(
            file="xml/ur10e_hand.xml",
            euler=(0, 0, 0),
        ),
    )

    entities["cube"] = scene.add_entity(
        material=gs.materials.Rigid(rho=300),
        morph=gs.morphs.Box(
            pos=(-0.25, 0.32, 0),
            size=(0.04, 0.04, 0.04),
        ),
        surface=gs.surfaces.Default(color=(0.5, 1, 0.5)),
    )

    entities["target"] = scene.add_entity(
        gs.morphs.Mesh(
            file="meshes/axis.obj",
            scale=0.15,
            collision=False,
        ),
        surface=gs.surfaces.Default(color=(1, 0.5, 0.5, 1)),
    )

    follower_camera = scene.add_camera(
        res=(640, 480),
        pos=(0.0, 2.0, 0.5),
        lookat=(0.0, 0.0, 0.5),
        fov=40,
        GUI=True
    )

    follower_camera.follow_entity(
        entities["cube"],
        fixed_axis=(None, None, 0.5),
        smoothing=0.5,
        fix_orientation=True
    )

    scene.build()
    return scene, entities


def run_sim(scene, entities, target_xy):
    robot = entities["robot"]
    target_entity = entities["target"]

    gx, gy = target_xy
    robot_init_pos = np.array([gx, gy, 0.3], dtype=np.float32)

    robot_init_R = R.from_euler("x", (55 * np.pi) / 36)
    target_pos = robot_init_pos.copy()
    target_R = robot_init_R

    n_dofs = robot.n_dofs
    motors_dof = np.arange(n_dofs - 2)
    ee_link = robot.get_link("wrist_3_link")

    print("\nRobot target position:", robot_init_pos)

    rclpy.init()
    node = Teleop()

    target_quat = target_R.as_quat(scalar_first=True)
    target_entity.set_qpos(np.concatenate([target_pos, target_quat]))

    q = robot.inverse_kinematics(link=ee_link, pos=target_pos, quat=target_quat)

    if q is None:
        print("IK failed. Could not find a solution.")
        node.destroy_node()
        rclpy.shutdown()
        return

    angles = q[:6]
    node.joints = angles.tolist()

    print("Sending robot once...")
    node.send()

    robot.control_dofs_position(q[:-2], motors_dof)

    for _ in range(300):
        rclpy.spin_once(node, timeout_sec=0.0)
        scene.step()

    print("Done. Robot moved once and stopped.")

    node.destroy_node()
    rclpy.shutdown()


def main():
    gx, gy = detect_cube_genesis_xy()

    if gx is None or gy is None:
        print("Could not detect cube.")
        return

    scene, entities = build_scene()
    run_sim(scene, entities, target_xy=(gx, gy))


if __name__ == "__main__":
    main()