import cv2 as cv 
from cv2 import aruco 
import os

# Create directory if it doesn't exist
output_dir = "/home/yongtak_ras/test/picamera/generate_markers/marker_images"
os.makedirs(output_dir, exist_ok=True)

marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
MARKER_SIZE = 400

# IDs to assign to markers
marker_ids = [
    "I1", "I2", "I3",
    "O1", "O2", "O3",
    "R1", "R2",
    "A1", "A2", "A3",
    "B1", "B2", "B3",
    "C1", "C2", "C3",
    "P1", "P2", "P3"
]

for id, marker_id in enumerate(marker_ids):
    marker_image = aruco.generateImageMarker(marker_dict, id, MARKER_SIZE)
    cv.imwrite(f"{output_dir}/marker_{marker_id}.png", marker_image)
