import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# Load YOLO model
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
        r = results[0]

        # Print x and y position for detected cube(s)
        for box in r.boxes:
            cls_id = int(box.cls[0])
            label = model.names[cls_id]

            if label == "cube":
                x, y, w, h = box.xywh[0]
                print(f"Cube position -> x: {float(x):.2f}, y: {float(y):.2f}")

        # Draw boxes/labels
        annotated = r.plot()

        cv2.imshow("YOLO (RealSense Color)", annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()