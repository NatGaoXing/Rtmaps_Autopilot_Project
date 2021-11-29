import rtmaps.types
import numpy as np
import rtmaps.core as rt 
import rtmaps.reading_policy 
from rtmaps.base_component import BaseComponent

import cv2

from queue import Queue
from queue import Empty

import os
import sys


# Perspective Transform from vehicle Camera to Bird's Eye View
def warp(img):
    img_size = (img.shape[1], img.shape[0])
    offset = 150

    # Source points taken from images with straight lane lines,
    # These are to become parallel after the warp transform
    src = np.float32([
        (0, 480),       # bottom-left corner
        (282, 160),     # top-left corner
        (358, 160),     # top-right corner
        (640, 480)      # bottom-right corner
    ])

    # Destination points are to be parallel, taken into account the image size
    dst = np.float32([
        [offset, img_size[1]],                  # bottom-left corner
        [offset, 0],                            # top-left corner
        [img_size[0] - offset, 0],              # top-right corner
        [img_size[0] - offset, img_size[1]]     # bottom-right corner
    ])

    # Calculate the transformation matrix and it's inverse transformation
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, img_size)

    return warped


def binary_thresholded(img):  # TODO Test with opencv on the real camera of AMI and in Simulation
    # Transform image to gray scale
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Apply sobel (derivative) in x direction, this is useful to detect lines that tend to be vertical
    sobelx = cv2.Sobel(gray_img, cv2.CV_64F, 1, 0)
    abs_sobelx = np.absolute(sobelx)
    # Scale result to 0-255
    scaled_sobel = np.uint8(255 * abs_sobelx / np.max(abs_sobelx))
    sx_binary = np.zeros_like(scaled_sobel)
    # Keep only derivative values that are in the margin of interest
    sx_binary[(scaled_sobel >= 30) & (scaled_sobel <= 255)] = 1

    # Detect pixels that are white in the grayscale image
    white_binary = np.zeros_like(gray_img)
    white_binary[(gray_img > 200) & (gray_img <= 255)] = 1  # 200,255

    # Convert image to HLS
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    H = hls[:, :, 0]
    S = hls[:, :, 2]
    sat_binary = np.zeros_like(S)
    # Detect pixels that have a high saturation value
    sat_binary[(S > 200) & (S <= 255)] = 1  # 90 , 255

    hue_binary = np.zeros_like(H)
    # Detect pixels that are yellow using the hue component
    hue_binary[(H > 15) & (H <= 25)] = 1  # 10, 25

    # Combine all pixels detected above
    binary_1 = cv2.bitwise_or(sx_binary, white_binary)
    binary_2 = cv2.bitwise_or(hue_binary, sat_binary)
    binary = cv2.bitwise_or(binary_1, binary_2)

    return binary


def draw_poly_lines(binary_warped):
    return np.dstack((binary_warped, binary_warped, binary_warped)) * 255


def lane_finding_pipeline(img):
    binary_thresh = binary_thresholded(img)
    binary_warped = warp(binary_thresh)

    draw_poly_img = draw_poly_lines(binary_warped)

    return draw_poly_img


def process_img(image):
    img_bird_view = lane_finding_pipeline(image)

    img_bird_view = cv2.resize(img_bird_view, None, fx=0.4, fy=1, interpolation=cv2.INTER_CUBIC)
    cv2.imshow('bird_view_img', img_bird_view)

    cv2.waitKey(1)

    return img_out


class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)

    def Dynamic(self):
        self.add_input("in", rtmaps.types.ANY)
        self.add_output("out", rtmaps.types.AUTO)

    def Birth(self):
        pass  # TODO Calibration of AMI's camera

    def Core(self):
        image_raw_data = self.inputs["in"].ioelt.data  # create an ioelt from the input

        im_array = np.copy(np.frombuffer(image_raw_data, dtype=np.dtype("uint8")))
        im_array = np.reshape(im_array, (480, 640, 4))

        im_array = im_array[:, :, :3][:, :, ::-1]

        try:
            process_img(im_array)
        except:
            pass

        tmpImage = rtmaps.types.IplImage()
        tmpImage.color_model = "RGB"
        tmpImage.channel_seq = "RGB"
        tmpImage.image_data = np.copy(im_array)

        self.outputs["out"].write(tmpImage)

    def Death(self):
        cv2.destroyAllWindows()
