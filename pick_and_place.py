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

    def send(self, move_time=5.0):
        if self.joints is None:
            return

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = self.joints
        pt.time_from_start.sec = int(move_time)
        pt.time_from_start.nanosec = int((move_time - int(move_time)) * 1e9)
        traj.points = [pt]

        self.pub.publish(traj)


def cam_to_genesis(x_cam, y_cam):
    x_gen = -0.5039 * x_cam + 3.0573 * y_cam - 0.1371
    y_gen =  0.8236 * x_cam + 1.4188 * y_cam + 0.2064
    return x_gen, y_gen


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

    # cube center should be at half its height, not 0
    cube_size = (0.04, 0.04, 0.04)
    cube_half_z = cube_size[2] / 2.0

    entities["cube"] = scene.add_entity(
        material=gs.materials.Rigid(rho=300),
        morph=gs.morphs.Box(
            pos=(-0.25, 0.32, cube_half_z),
            size=cube_size,
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

    scene.build()
    return scene, entities


def move_robot_once(scene, entities, node, target_xyz, move_time=5.0):
    robot = entities["robot"]
    target_entity = entities["target"]

    target_pos = np.array(target_xyz, dtype=np.float32)
    target_R = R.from_euler("x", (55 * np.pi) / 36)

    n_dofs = robot.n_dofs
    motors_dof = np.arange(n_dofs - 2)
    ee_link = robot.get_link("wrist_3_link")

    target_quat = target_R.as_quat(scalar_first=True)
    target_entity.set_qpos(np.concatenate([target_pos, target_quat]))

    q = robot.inverse_kinematics(link=ee_link, pos=target_pos, quat=target_quat)

    if q is None:
        print("IK failed for target:", target_pos)
        return False

    angles = q[:6]
    node.joints = angles.tolist()

    print("Sending robot to:", target_pos)
    node.send(move_time=move_time)
    robot.control_dofs_position(q[:-2], motors_dof)

    steps = int(move_time * 60) + 60
    for _ in range(steps):
        rclpy.spin_once(node, timeout_sec=0.0)
        scene.step()

    return True


def detect_cube_genesis_xy_realsense(model):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    align = rs.align(rs.stream.color)

    print("Waiting for cube detection... Press q to quit.")

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
                cv2.waitKey(500)

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


def main():
    model = YOLO("/home/credit-research1/SIM-to-RELA/my_model.pt")

    scene, entities = build_scene()

    rclpy.init()
    node = Teleop()

    ROBOT_IP = "192.168.56.101"  # <-- change this
    g = RobotiqHandE_URCapSocket(ROBOT_IP)

    g.connect()
    g.open()
    # Move to initial waiting position first
    initial_xyz = (-0.2, 0.02, 1.0)
    ok = move_robot_once(scene, entities, node, initial_xyz, move_time=5.0)
    if not ok:
        print("Could not move to initial position.")
        node.destroy_node()
        rclpy.shutdown()
        return

    gx, gy = detect_cube_genesis_xy_realsense(model)

    if gx is None or gy is None:
        print("Could not detect cube.")
        node.destroy_node()
        rclpy.shutdown()
        return

    # ----- simple pick/place style sequence -----
    # Use hover points instead of jumping directly to place z=0
    pre_grasp_xyz = (gx, gy, 0.30)
    grasp_xyz     = (gx, gy, 0.25)   # tune this if needed
    lift_xyz      = (gx, gy, 0.30)

    pre_place_xyz = (-0.45, 0.218, 0.30)
    place_xyz     = (-0.45, 0.218, 0.25)  # do NOT use z=0 for wrist_3_link
    post_place_xyz = (-0.45, 0.218, 0.30)

    # approach cube
    ok = move_robot_once(scene, entities, node, pre_grasp_xyz, move_time=4.0)
    if not ok:
        print("Could not move to pre-grasp.")
        node.destroy_node()
        rclpy.shutdown()
        return

    ok = move_robot_once(scene, entities, node, grasp_xyz, move_time=3.0)
    if not ok:
        print("Could not move to grasp.")
        node.destroy_node()
        rclpy.shutdown()
        return

    # NOTE:
    # This script still does not actually close/open a gripper.
    # So the robot motion is fixed, but true placing requires gripper logic.
    print("Reached grasp position. Add gripper close here if needed.")
    g.close_grip()

    ok = move_robot_once(scene, entities, node, lift_xyz, move_time=3.0)
    if not ok:
        print("Could not lift from cube.")
        node.destroy_node()
        rclpy.shutdown()
        return

    ok = move_robot_once(scene, entities, node, pre_place_xyz, move_time=4.0)
    if not ok:
        print("Could not move to pre-place.")
        node.destroy_node()
        rclpy.shutdown()
        return

    ok = move_robot_once(scene, entities, node, place_xyz, move_time=3.0)
    if not ok:
        print("Could not move to place.")
        node.destroy_node()
        rclpy.shutdown()
        return

    print("Reached place position. Add gripper open here if needed.")
    g.open()
    ok = move_robot_once(scene, entities, node, post_place_xyz, move_time=3.0)
    if not ok:
        print("Could not lift after place.")
        node.destroy_node()
        rclpy.shutdown()
        return

    print("Done.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()