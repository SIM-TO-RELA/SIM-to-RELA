import os
import random
import threading

import genesis as gs
import numpy as np
from pynput import keyboard
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from pynput import keyboard
import math
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

from test_gripper import RobotiqHandE_URCapSocket

class KeyboardDevice:
    def __init__(self):
        self.pressed_keys = set()
        self.lock = threading.Lock()
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)

    def start(self):
        self.listener.start()

    def stop(self):
        try:
            self.listener.stop()
        except NotImplementedError:
            # Dummy backend does not implement stop
            pass
        self.listener.join()

    def on_press(self, key: keyboard.Key):
        with self.lock:
            self.pressed_keys.add(key)

    def on_release(self, key: keyboard.Key):
        with self.lock:
            self.pressed_keys.discard(key)

    def get_cmd(self):
        return self.pressed_keys
    

def build_scene():

    
    ########################## init ##########################
    gs.init(precision="32", logging_level="info", backend=gs.cpu)
    np.set_printoptions(precision=7, suppress=True)

    ########################## create a scene ##########################
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
            camera_pos=(1.5, 0.0, 0.7),
            camera_lookat=(0.2, 0.0, 0.1),
            camera_fov=50,
            max_FPS=60,
        ),
        show_viewer=True,
        show_FPS=False,
    )

    entities = dict()
    entities["plane"] = scene.add_entity(
        gs.morphs.Plane(),
    )

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
            pos=(0.5, 0.0, 0.07),
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


    follower_camera = scene.add_camera(res=(640,480),
                                        pos=(0.0, 2.0, 0.5),
                                        lookat=(0.0, 0.0, 0.5),
                                        fov=40,
                                        GUI=True)
 
    follower_camera.follow_entity(entities["cube"], fixed_axis=(None, None, 0.5), smoothing=0.5, fix_orientation=True)
    ########################## build ##########################
    scene.build()

    # rclpy.init()
    # node = Teleop()
    # joints = node.js_cb
    # rclpy.spin(node)

    return scene, entities 

def run_sim(scene, entities, clients):
    robot = entities["robot"]
    target_entity = entities["target"]

    robot_init_pos = np.array([-0.4, 0, 1.25])
    robot_init_R = R.from_euler("x", (55*np.pi)/36)
    target_pos = robot_init_pos.copy()
    target_R = robot_init_R

    n_dofs = robot.n_dofs
    motors_dof = np.arange(n_dofs-2)
    # print(motors_dof)
    fingers_dof = np.array([6,7])

    ee_link = robot.get_link("wrist_3_link")
 

    def reset_scene():
        nonlocal target_pos, target_R
        target_pos = robot_init_pos.copy()
        target_R = robot_init_R
        target_quat = target_R.as_quat(scalar_first=True)
        target_entity.set_qpos(np.concatenate([target_pos, target_quat]))
        q = robot.inverse_kinematics(link=ee_link, pos=target_pos, quat=target_quat)
        robot.set_qpos(q[:-2], motors_dof)

        entities["cube"].set_pos((random.uniform(0.2, 0.4), random.uniform(-0.2, 0.2), 0.05))
        entities["cube"].set_quat(R.from_euler("z", random.uniform(0, np.pi * 2)).as_quat(scalar_first=True))

       

    "code ../miniconda3/envs/genesis_env/lib/python3.12/site-packages/genesis/ext/pyrender/viewer.py  - THIS IS UI CONTROL"

    print("\nKeyboard Controls:")
    print("↑\t- Move Forward (North)")
    print("↓\t- Move Backward (South)")
    print("←\t- Move Left (West)")
    print("→\t- Move Right (East)")
    print("n\t- Move Up")
    print("m\t- Move Down")
    print("j\t- Rotate Counterclockwise")
    print("k\t- Rotate Clockwise")
    print("u\t- Reset Scene")
    print("space\t- Press to close gripper, release to open gripper")
    print("esc\t- Quit")

    # reset scen before starting teleoperation
    reset_scene()

    # start teleoperation
    
    rclpy.init()
    node = Teleop()
    

    stop = False

    while not stop:

        rclpy.spin_once(node, timeout_sec=0.0)
        pressed_keys = clients["keyboard"].pressed_keys.copy()

        # reset scene:
        reset_flag = False
        reset_flag |= keyboard.KeyCode.from_char("u") in pressed_keys
        if reset_flag:
            reset_scene()

        # stop teleoperation
        stop = keyboard.Key.esc in pressed_keys

        # get ee target pose
        is_close_gripper = False
        dpos = 0.002
        drot = 0.01
        
        ROBOT_IP = "192.168.56.101"  # <-- change this
        g = RobotiqHandE_URCapSocket(ROBOT_IP)

        g.connect()
        
        for key in pressed_keys:
            if key == keyboard.Key.up:
                target_pos[0] -= dpos
            elif key == keyboard.Key.down:
                target_pos[0] += dpos
            elif key == keyboard.Key.right:
                target_pos[1] += dpos
            elif key == keyboard.Key.left:
                target_pos[1] -= dpos
            elif key == keyboard.KeyCode.from_char("n"):
                target_pos[2] += dpos
            elif key == keyboard.KeyCode.from_char("m"):
                target_pos[2] -= dpos
            elif key == keyboard.KeyCode.from_char("j"):
                target_R = R.from_euler("z", drot) * target_R
            elif key == keyboard.KeyCode.from_char("k"):
                target_R = R.from_euler("z", -drot) * target_R
            elif key == keyboard.Key.space:
                is_close_gripper = True

        # control arm
        arm_names = [
                "shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
                "wrist_1_joint","wrist_2_joint","wrist_3_joint"
            ]
        
       

        target_quat = target_R.as_quat(scalar_first=True)
        target_entity.set_qpos(np.concatenate([target_pos, target_quat]))
        q= robot.inverse_kinematics(link=ee_link, pos=target_pos, quat=target_quat)
        angles = q[:6]

        # for i in range(6):
        #     joint_angles[i] = angles[i]
        node.joints = angles.tolist()
        node.send()

        
        
        

        robot.control_dofs_position(q[:-2], motors_dof)
        # robot.set_qpos(q)

        s = 100000000000000
        # control gripper
        if is_close_gripper:
            robot.control_dofs_position(np.array([s, s]), fingers_dof)
            g.close_grip()
        else:
            robot.control_dofs_position(np.array([-s, -s]), fingers_dof)
            g.open()

        scene.step()

        if "PYTEST_VERSION" in os.environ:
            break

    node.destroy_node()
    rclpy.shutdown()

JOINT_NAMES = [
"shoulder_pan_joint",
"shoulder_lift_joint",
"elbow_joint",
"wrist_1_joint",
"wrist_2_joint",
"wrist_3_joint",
]

STEP = 0.05  # radians (~3 deg)

class Teleop(Node):
    def __init__(self):
        super().__init__('xyz_step')
        self.pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # self.base_frame = "base_link"
        # self.ee_frame = "wrist_3_link"   


        self.joints = None
        self.create_subscription(JointState, '/joint_states', self.js_cb, 10)

        # self.listener = keyboard.Listener(on_press=self.on_press)
        # self.listener.start()

        # print("\nControls:")
        # print("1/2 → joint1")
        # print("3/4 → joint2")
        # print("5/6 → joint3")
        # print("7/8 → joint4")
        # print("9/0 → joint5")
        # print("-/= → joint6\n")

    def js_cb(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))
        self.joints = [name_to_pos[j] for j in JOINT_NAMES]
        # return self.joints

    def send(self):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = self.joints
        pt.time_from_start.sec = 1
        traj.points = [pt]

        self.pub.publish(traj)






def main():

    

    clients = dict()
    clients["keyboard"] = KeyboardDevice()
    clients["keyboard"].start()

    scene, entities= build_scene()
    run_sim(scene, entities, clients)


if __name__ == "__main__":
    main()
