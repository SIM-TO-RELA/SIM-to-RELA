import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# Load YOLO model
model = YOLO("my_model.pt")

# Start RealSense color + depth streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# Align depth to color frame
align = rs.align(rs.stream.color)

print("Running YOLO + RealSense. Press 'q' to quit.")

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Convert RealSense frame to numpy array
        frame = np.asanyarray(color_frame.get_data())

        # Run YOLO
        results = model.predict(source=frame, verbose=False)
        r = results[0]

        # Get camera intrinsics for 3D conversion
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

        for box in r.boxes:
            cls_id = int(box.cls[0])
            label = model.names[cls_id]
            conf = float(box.conf[0])

            if label == "cube":
                x, y, w, h = box.xywh[0]

                # Center pixel of bounding box
                px = int(float(x))
                py = int(float(y))

                # Depth at center pixel (meters)
                depth = depth_frame.get_distance(px, py)

                # Only compute 3D point if depth is valid
                if depth > 0:
                    X, Y, Z = rs.rs2_deproject_pixel_to_point(intrinsics, [px, py], depth)

                    print(
                        f"Cube pixel -> x: {px}, y: {py} | "
                        f"3D position -> X: {X:.4f}, Y: {Y:.4f}, Z: {Z:.4f} m | "
                        f"conf: {conf:.2f}"
                    )

                    # Draw center point on image
                    cv2.circle(frame, (px, py), 5, (0, 255, 0), -1)
                    cv2.putText(
                        frame,
                        f"X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f}",
                        (px + 10, py - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2
                    )
                else:
                    print(f"Cube pixel -> x: {px}, y: {py} | depth invalid | conf: {conf:.2f}")

        # Draw YOLO boxes/labels
        annotated = r.plot()

        # If you want both YOLO boxes and the 3D point text, use frame instead
        # after drawing YOLO boxes onto it:
        annotated = r.plot(img=frame)

        cv2.imshow("YOLO + RealSense 3D", annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()