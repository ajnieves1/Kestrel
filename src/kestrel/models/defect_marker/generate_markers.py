# Generate the four ArUco stand in defect marker PNGs used on the pylon
import os

import cv2
import numpy as np

IMAGE_SIZE = 512
MARKER_SIZE = 400
MARGIN = (IMAGE_SIZE - MARKER_SIZE) // 2
MARKER_COUNT = 4


# Write marker_0.png through marker_3.png next to this script
def main():
    output_directory = os.path.dirname(os.path.abspath(__file__))
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    for marker_id in range(MARKER_COUNT):
        # generateImageMarker needs OpenCV 4.7, the image pins 4.6, drawMarker is the same call
        marker_image = cv2.aruco.drawMarker(dictionary, marker_id, MARKER_SIZE)
        canvas = np.full((IMAGE_SIZE, IMAGE_SIZE), 255, dtype=np.uint8)
        canvas[MARGIN:MARGIN + MARKER_SIZE, MARGIN:MARGIN + MARKER_SIZE] = marker_image
        output_path = os.path.join(output_directory, f'marker_{marker_id}.png')
        cv2.imwrite(output_path, canvas)
        print(f'wrote {output_path}')


if __name__ == '__main__':
    main()
