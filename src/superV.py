# superV.py

from ultralytics import YOLO
import supervision as sv  # Ensure this library is installed
import cv2
import numpy as np

# Load YOLO model
model = YOLO("yolov8n.pt")

print("Import Success")
# def process_frame(frame: np.ndarray) -> None:
#     """
#     This function processes a frame using YOLO for object detection and Supervision for annotation.
#     It receives the frame from C++ as a NumPy array.
#     """
#     # Perform YOLO detection
#     print("Process Start, From python!\n")
#     print(f"Original frame shape: {frame.shape}\n")
#     results = model(frame)[0]
#     detections = sv.Detections.from_ultralytics(results)

#     # Annotate the frame using Supervision
#     box_annotator = sv.BoxAnnotator()
#     label_annotator = sv.LabelAnnotator()

#     # Add annotations to the frame
#     box_annotator.annotate(scene=frame, detections=detections)
#     label_annotator.annotate(scene=frame, detections=detections)

def process_yuv_frame(y_plane, u_plane, v_plane, width, height):
    """
    Processes a frame for object detection and annotation, directly modifying the frame.
    """

    # Step 1: Merge Y, U, and V planes into a single YUV420 image
    yuv_image = np.zeros((height + height // 2, width), dtype=np.uint8)

    # Y plane is full resolution
    yuv_image[:height, :] = y_plane

    # U and V planes are half resolution in both width and height
    u_height = height // 2
    u_width = width // 2

    # Correctly placing U and V planes in the YUV420 frame
    yuv_image[height:height + u_height, :u_width] = u_plane
    yuv_image[height + u_height:, :u_width] = v_plane

    # Step 2: Convert YUV420 to BGR for processing with Supervision/YOLO
    bgr_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_I420)

    # Step 3: Perform YOLO detection (object detection)
    results = model(bgr_image)[0]
    detections = sv.Detections.from_ultralytics(results)

    # Step 4: Annotate the BGR image with bounding boxes and labels using Supervision
    box_annotator = sv.BoxAnnotator()
    label_annotator = sv.LabelAnnotator()
    box_annotator.annotate(scene=bgr_image, detections=detections)
    label_annotator.annotate(scene=bgr_image, detections=detections)

    # Step 5: Convert the annotated BGR image back to YUV420
    modified_yuv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2YUV_I420)

    # Step 6: Write the modified YUV planes back to the original memory (C++ AVFrame)
    y_plane[:, :] = modified_yuv_image[:height, :]
    u_plane[:, :] = modified_yuv_image[height:height + u_height, :u_width]
    v_plane[:, :] = modified_yuv_image[height + u_height:, :u_width]