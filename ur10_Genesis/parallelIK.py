import os
os.environ['PYOPENGL_PLATFORM'] = 'glx'
"export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 --  ADD THIS BEOFRE RUNNING SCRIPT"
import genesis as gs
import numpy as np
from genesis.constants import backend as gs_backend
import torch

gs.init(gs_backend.cpu)

scene = gs.Scene(
    show_viewer    = True,
    viewer_options = gs.options.ViewerOptions(
        camera_pos    = (3.5, -1.0, 2.5),
        camera_lookat = (0.0, 0.0, 0.5),
        camera_fov    = 40,
    ),
    rigid_options = gs.options.RigidOptions(
        dt                = 0.01,
    ),
)


plane = scene.add_entity(
    gs.morphs.Plane(),
)

robot = scene.add_entity(
    gs.morphs.MJCF(file='ur10e_2f85.xml',pos=(0.0, 0.0, 0.0), visualization = True),
)

# cam = scene.add_camera(
#     # model = 'thinlens',
#     res    = (640, 480),
#     pos    = (0, -5.75, 2),
#     lookat = (0, 0, 0.5),
#     fov    = 40,
#     GUI    = True,
# )

# scene.build()

n_envs = 20
scene.build(n_envs=n_envs, env_spacing=(1.0, 1.0))
target_quat = np.tile(np.array([0, 1, 0, 0]), [n_envs, 1]) # pointing downwards
center = np.tile(np.array([0.4, -0.2, 0.25]), [n_envs, 1])
angular_speed = np.random.uniform(-10, 10, n_envs)
r = 0.1

ee_link = robot.get_link('shoulder_link')


for i in range(0, 1000):
    target_pos = np.zeros([n_envs, 3])
    target_pos[:, 0] = center[:, 0] + np.cos(i/360*np.pi*angular_speed) * r
    target_pos[:, 1] = center[:, 1] + np.sin(i/360*np.pi*angular_speed) * r
    target_pos[:, 2] = center[:, 2]
    target_q = np.hstack([target_pos, target_quat])

    q = robot.inverse_kinematics(
        link     = ee_link,
        pos      = target_pos,
        quat     = target_quat,
        rot_mask = [False, False, True], # for demo purpose: only restrict direction of z-axis
    )

    robot.set_qpos(q)
    scene.step()