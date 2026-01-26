import os
import random
import threading

import genesis as gs
import numpy as np
from pynput import keyboard
from scipy.spatial.transform import Rotation as R



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
            file="ur10e_roboti.xml",
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

    return scene, entities

def run_sim(scene, entities, clients):
    robot = entities["robot"]
    target_entity = entities["target"]

    robot_init_pos = np.array([0.5, 0, 0.55])
    robot_init_R = R.from_euler("x", (55*np.pi)/36)
    target_pos = robot_init_pos.copy()
    target_R = robot_init_R

    n_dofs = robot.n_dofs
    motors_dof = np.arange(n_dofs-8)
    print(motors_dof)
    fingers_dof = np.array([6,8])

    ee_link = robot.get_link("wrist_3_link")
 

    joint_name="shoulder_pan_joint"
    jp = robot.get_dofs_position()
    angle = jp[0]
    print(angle)
    def reset_scene():
        nonlocal target_pos, target_R
        target_pos = robot_init_pos.copy()
        target_R = robot_init_R
        target_quat = target_R.as_quat(scalar_first=True)
        target_entity.set_qpos(np.concatenate([target_pos, target_quat]))
        q = robot.inverse_kinematics(link=ee_link, pos=target_pos, quat=target_quat)
        robot.set_qpos(q[:-8], motors_dof)

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
    
    stop = False
    while not stop:
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
        
        robot.control_dofs_position(q[:-8], motors_dof)
        # robot.set_qpos(q)

        s = 10
        # control gripper
        if is_close_gripper:
            robot.control_dofs_position(np.array([s, s]), fingers_dof)
        else:
            robot.control_dofs_position(np.array([-s, -s]), fingers_dof)

        scene.step()

        if "PYTEST_VERSION" in os.environ:
            break


def main():
    clients = dict()
    clients["keyboard"] = KeyboardDevice()
    clients["keyboard"].start()

    scene, entities = build_scene()
    run_sim(scene, entities, clients)


if __name__ == "__main__":
    main()
