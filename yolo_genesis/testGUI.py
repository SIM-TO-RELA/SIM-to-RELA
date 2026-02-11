import os
os.environ['PYOPENGL_PLATFORM'] = 'glx'
"export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 --  ADD THIS BEOFRE RUNNING SCRIPT"
import genesis as gs
import numpy as np
from genesis.constants import backend as gs_backend
import torch
import cv2
from ultralytics import YOLO    

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
cam= scene.add_camera(
            pos=(1.00, 3.25, 1),
            lookat=(0.0, 0.0, 0.0),
            fov=60,
            GUI=True,
        )
plane = scene.add_entity(
    gs.morphs.Plane(),
)

robot = scene.add_entity(
    gs.morphs.MJCF(file='../xml/ur10e_roboti.xml',pos=(0.0, 0.0, 0.0), visualization = True),
)

tv = scene.add_entity(
    gs.morphs.MJCF(file='../../3Dstuff/3D_files/tv.xml',pos=(0.0, 2.0, 0.0), visualization = True),
)






scene.build()

model =YOLO("yolo11n_ncnn_model", task = 'detect')
labels = model.names

bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106), 
        (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]

cap = cam
# cap.configure(cap.create_video_configuration(main={"format": 'XRGB8888', "size": (1280, 720)}))
# cap.start()
frame_bgra = cap.render(rgb=True)
for index in frame_bgra:
    frame = cv2.cvtColor(np.copy(index), cv2.COLOR_RGB2BGR)
    results = model(frame, verbose=False)
cv2.waitKey(1)


# Extract results
detections = results[0].boxes


# Initialize variable for basic object counting example
object_count = 0

target=[]
spawned = []

for i in range(len(detections)):
# for i in range(15):
# print(i)
# Get bounding box coordinates
# Ultralytics returns results in Tensor format, which have to be converted to a regular Python array
    xyxy_tensor = detections[i].xyxy.cpu() # Detections in Tensor format in CPU memory
    xyxy = xyxy_tensor.numpy().squeeze() # Convert tensors to Numpy array
    xmin, ymin, xmax, ymax = xyxy.astype(int) # Extract individual coordinates and convert to int

    # Get bounding box class ID and name
    classidx = int(detections[i].cls.item())
    classname = labels[classidx]

    conf = detections[i].conf.item()

    # Draw box if confidence threshold is high enough
    if conf > 0.5:

        color = bbox_colors[classidx % 10]
        cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), color, 2)

        label = f'{classname}: {int(conf*100)}%'
        labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1) # Get font size
        label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
        cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), color, cv2.FILLED) # Draw white box to put label text in
        cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1) # Draw label text

        # Basic example: count the number of objects in the image
        object_count = object_count + 1




for i in range(100000):
    scene.step()
