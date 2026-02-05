import os
os.environ['PYOPENGL_PLATFORM'] = 'glx'
"export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 --  ADD THIS BEOFRE RUNNING SCRIPT"
import genesis as gs
import numpy as np
from genesis.constants import backend as gs_backend
import torch

gs.init(gs_backend.cpu)

scene = gs.Scene(
    show_viewer = True,
    rigid_options=gs.options.RigidOptions(enable_joint_limit = False,enable_collision=False, gravity=(0, 0, 0)),
    viewer_options= gs.options.ViewerOptions(
        camera_pos    = (0.0, -2, 1.5),
        camera_lookat = (0.0, 0.0, 0.5),
        camera_fov    = 40,
        max_FPS       = 60,
    ),
    
    vis_options = gs.options.VisOptions(
        show_world_frame = True,
        world_frame_size = 1.0,
        show_link_frame  = False,
        show_cameras     = False,
        plane_reflection = True,
        ambient_light    = (0.1, 0.1, 0.1),
        background_color = (1,1,1),
    ),
    renderer=gs.renderers.Rasterizer(),
    
)

plane = scene.add_entity(
    gs.morphs.Plane(),
)

bottle2 = scene.add_entity(
    gs.morphs.MJCF(file='ur10e_hand.xml',pos=(0.0, 0.0, 0.0), visualization = True),
)




# cam = scene.add_camera(
#     # model = 'thinlens',
#     res    = (640, 480),
#     pos    = (0, -5.75, 2),
#     lookat = (0, 0, 0.5),
#     fov    = 40,
#     GUI    = True,
# )

scene.build()

# position= (
#     "bottle"
# )

# bottle.set_pos([0,2,0]) 

# print(pos)

for i in range(1000):


    # target_pos = np.zeros(3)
    # target_pos[:,0] = center[0] + np.cos(i/360*np.pi*angular_speed) * r
    # target_pos[:,1] = center[1] + np.sin(i/360*np.pi*angular_speed) * r
    # target_pos[:,2] = center[2]
    # target_q = np.hstack([target_pos, target_quat])

    # q = robot.inverse_kinematics(
    #     link    = end_effector,
    #     pos      = np.array([ 0.4 , -0.2 +i, 0.25]),
    #     quat    = np.tile([0,0] , i),
    #     rot_mask = [False, False, True], # for demo purpose: only restrict direction of z-axis
    # )

    # z = robot.inverse_kinematics(
    #     link    = link2,
    #     pos      = np.array([ 0.4+i , -0.2 +i, 0.25+i]),
    #     quat    = np.tile([0,0] , i),
    #     rot_mask = [False, False, True], # for demo purpose: only restrict direction of z-axis
    # )

    # try:
    #     robot.control_dofs_position(path[i])
    #     last_pt = path[i]
    # except:
    #     robot.control_dofs_position(last_pt)
    # robot.set_qpos(q)
    # robot.set_qpos(z)
    scene.step()
    # cam.set_pose(pos=(i / 100, 0, 2.5))
    # cam.render()
# cam.stop_recording(save_to_filename='/raid/credit/pande/genesis/robot_arm/vids/ur10.mp4', fps=60)
