import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# Load a small YOLO model (fast)
model = YOLO("my_model.pt")

# Start RealSense color stream
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

print("Running YOLO + RealSense. Press 'q' to quit.")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())  # BGR image

        # Run YOLO on this frame
        results = model.predict(source=frame, verbose=False)
        print(results)
        annotated = results[0].plot()  # draws boxes/labels

        cv2.imshow("YOLO (RealSense Color)", annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
