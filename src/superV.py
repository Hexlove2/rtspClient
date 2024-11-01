# superV.py

from ultralytics import YOLO
import supervision as sv  # Ensure this library is installed
import cv2
import numpy as np

# Load YOLO model
model = YOLO("yolov8n.pt")

print("Import Success")


def process_yuv_frame(y_plane, u_plane, v_plane, width, height):

    y_plane = y_plane.reshape((height, width))
    u_plane = u_plane.reshape((height // 2, width // 2))
    v_plane = v_plane.reshape((height // 2, width // 2))


    #print("Start convert")
    u_up = cv2.resize(u_plane, (width, height), interpolation=cv2.INTER_LINEAR)
    v_up = cv2.resize(v_plane, (width, height), interpolation=cv2.INTER_LINEAR) 

    # 确保数据类型一致
    u_up = u_up.astype(y_plane.dtype)
    v_up = v_up.astype(y_plane.dtype)

    bgr_image = cv2.merge([y_plane, u_up, v_up])
    bgr_image = cv2.cvtColor(bgr_image, cv2.COLOR_YUV2BGR)
    cv2.imwrite("bgr.jpg", bgr_image)
    
    # YOLO Detection and Supervision annotations
    #frame_real_process(bgr_image)

    # Convert annotated image back to YUV420 format
    cv2.imwrite("bgr2.jpg", bgr_image)

    modified_yuv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2YUV_I420)

    cv2.imwrite("yuv.jpg", modified_yuv_image)

    # Assign modified planes back to original buffers
    # print("Modified YUV image shape:", modified_yuv_image.shape)  # Expecting (height * 3 // 2, width)
    # print("Modified YUV image size:", modified_yuv_image.size)    # Expecting height * width * 3 // 2
    # print("height:",height,"width",width, "\n")

    y_plane[:,:] = modified_yuv_image[:height, :].reshape((height, width))
    u_plane[:,:] = modified_yuv_image[height:height+height//4, :width].reshape((height//2, width//2))
    v_plane[:,:] = modified_yuv_image[height+height//4:height+height//2, :width].reshape((height//2, width//2))
    return y_plane.copy(), u_plane.copy(), v_plane.copy()

def frame_real_process(bgr_image):
    # YOLO Detection and Supervision annotations
    results = model(bgr_image)[0]
    detections = sv.Detections.from_ultralytics(results)
    box_annotator = sv.BoxAnnotator()
    label_annotator = sv.LabelAnnotator()
    box_annotator.annotate(scene=bgr_image, detections=detections)
    label_annotator.annotate(scene=bgr_image, detections=detections)