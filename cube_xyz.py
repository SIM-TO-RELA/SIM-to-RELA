import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO


class CubeXYZ:
    def __init__(self, model_path):
        self.model = YOLO(model_path)

    @staticmethod
    def cam_to_genesis(x_cam, y_cam):
        x_gen = -0.5039 * x_cam + 3.0573 * y_cam - 0.1371
        y_gen =  0.8236 * x_cam + 1.4188 * y_cam + 0.2064
        return x_gen, y_gen

    def get_genesis_xyz(self, show=False):
        # 🔥 CAMERA STARTS HERE AUTOMATICALLY
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        pipeline.start(config)
        align = rs.align(rs.stream.color)

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

                results = self.model.predict(source=frame, verbose=False)
                r = results[0]

                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    label = self.model.names[cls_id]

                    if label != "cube":
                        continue

                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

                    px = int((x1 + x2) / 2)
                    py = int((y1 + y2) / 2)

                    depth = depth_frame.get_distance(px, py)
                    if depth <= 0:
                        continue

                    X, Y, Z = rs.rs2_deproject_pixel_to_point(
                        intrinsics, [px, py], depth
                    )

                    gx, gy = self.cam_to_genesis(X, Y)

                    if show:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                        cv2.circle(frame, (px, py), 5, (0, 255, 0), -1)
                        cv2.imshow("Detection", frame)
                        cv2.waitKey(1)

                    return gx, gy, Z

                if show:
                    cv2.imshow("Detection", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        return None

        finally:
            # 🔥 CAMERA STOPS AUTOMATICALLY
            pipeline.stop()
            cv2.destroyAllWindows()