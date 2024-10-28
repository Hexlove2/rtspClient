# superV.py

from ultralytics import YOLO
import supervision as sv  # Ensure this library is installed
import cv2
import ctypes
import numpy as np

# Load YOLO model
model = YOLO("yolov8n.pt")

print("Import Success")


def process_yuv_frame(y_plane, u_plane, v_plane, width, height):
    """
    Processes a frame for object detection and annotation, directly modifying the frame.
    """
    # y_plane = np.ctypeslib.as_array((ctypes.c_uint8 * (width * height)).from_address(y_ptr))
    # u_plane = np.ctypeslib.as_array((ctypes.c_uint8 * (width // 2 * height // 2)).from_address(u_ptr))
    # v_plane = np.ctypeslib.as_array((ctypes.c_uint8 * (width // 2 * height // 2)).from_address(v_ptr))

    y_plane = y_plane.reshape((height, width))
    u_plane = u_plane.reshape((height // 2, width // 2))
    v_plane = v_plane.reshape((height // 2, width // 2))


    print("Start convert")
    # yuv_rawImage = np.zeros((height*3, width), dtype=np.uint8)
    # yuv_rawImage[:height,:] = y_plane
    # yuv_rawImage[height:height + height , :width] = u_plane.repeat(2, axis=1).repeat(2,axis=0)
    # yuv_rawImage[height+height:height+height+height, :width] = v_plane.repeat(2, axis=1).repeat(2,axis=0)
    # u_up=u_plane.repeat(2, axis=1).repeat(2,axis=0).copy()
    # v_up=v_plane.repeat(2, axis=1).repeat(2,axis=0).copy()
    u_up = cv2.resize(u_plane, (width, height), interpolation=cv2.INTER_LINEAR)
    v_up = cv2.resize(v_plane, (width, height), interpolation=cv2.INTER_LINEAR) 

    # 确保数据类型一致
    u_up = u_up.astype(y_plane.dtype)
    v_up = v_up.astype(y_plane.dtype)

    #yuv_rawImage[height:height+height//2, :width//2] = u_plane
    #yuv_rawImage[height:height+height//2, width//2:width] = v_plane

    # Convert YUV420 to BGR for processing with YOLO

    bgr_image = cv2.merge([y_plane, u_up, v_up])
    bgr_image = cv2.cvtColor(bgr_image, cv2.COLOR_YUV2BGR)
    cv2.imwrite("bgr.jpg", bgr_image)
    
    # YOLO Detection and Supervision annotations
    results = model(bgr_image)[0]
    detections = sv.Detections.from_ultralytics(results)
    box_annotator = sv.BoxAnnotator()
    label_annotator = sv.LabelAnnotator()
    box_annotator.annotate(scene=bgr_image, detections=detections)
    label_annotator.annotate(scene=bgr_image, detections=detections)

    # Convert annotated image back to YUV420 format
    cv2.imwrite("bgr2.jpg", bgr_image)

    modified_yuv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2YUV_I420)

    cv2.imwrite("yuv.jpg", modified_yuv_image)

    # Assign modified planes back to original buffers
    print("Modified YUV image shape:", modified_yuv_image.shape)  # Expecting (height * 3 // 2, width)
    print("Modified YUV image size:", modified_yuv_image.size)    # Expecting height * width * 3 // 2
    print("height:",height,"width",width, "\n")

    y_plane[:,:] = modified_yuv_image[:height, :].reshape((height, width))
    u_plane[:,:] = modified_yuv_image[height:height+height//4, :width].reshape((height//2, width//2))
    v_plane[:,:] = modified_yuv_image[height+height//4:height+height//2, :width].reshape((height//2, width//2))
    return y_plane.copy(), u_plane.copy(), v_plane.copy()