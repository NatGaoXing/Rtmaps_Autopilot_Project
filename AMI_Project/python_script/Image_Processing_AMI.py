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

import pickle


def preProcessing(_img):
    _img = cv2.cvtColor(_img, cv2.COLOR_BGR2GRAY)
    _img = cv2.equalizeHist(_img)
    _img = _img / 255
    return _img


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

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
    kernel1 = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
    img_Erode_warped = cv2.erode(warped, kernel)
    img_Dialate_warped = cv2.dilate(img_Erode_warped, kernel1)

    return img_Dialate_warped


def detect_warp(img):
    img_size = (img.shape[1], img.shape[0])
    offset = 150

    src = np.float32([
        (150, 320),     # bottom-left corner
        (150, 160),     # top-left corner
        (120, 160),     # top-right corner
        (120, 320)      # bottom-right corner
    ])

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


def detect_warp_sign(img, bl, tl, tr, br):
    # print(bl, tl, tr, br)
    img_size = (img.shape[1], img.shape[0])
    offset = 0

    src = np.float32([
        bl,  # bottom-left corner
        tl,  # top-left corner
        tr,  # top-right corner
        br  # bottom-right corner
    ])

    dst = np.float32([
        [offset, img_size[1]],  # bottom-left corner
        [offset, 0],  # top-left corner
        [img_size[0] - offset, 0],  # top-right corner
        [img_size[0] - offset, img_size[1]]  # bottom-right corner
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
    white_binary[(gray_img > 220) & (gray_img <= 255)] = 1  # 200,255

    # Convert image to HLS
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    H = hls[:, :, 0]
    S = hls[:, :, 2]
    sat_binary = np.zeros_like(S)
    # Detect pixels that have a high saturation value
    sat_binary[(S > 200) & (S <= 255)] = 1  # 90 , 255

    hue_binary = np.zeros_like(H)
    # Detect pixels that are yellow using the hue component
    hue_binary[(H > 65) & (H <= 80)] = 1  # 10, 25

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


def add_mask(img, p, r):
    mask = np.zeros(img.shape[:2], dtype=np.uint8)
    mask = cv2.circle(mask, p, r, (255, 255, 255), -1)

    return mask


def detect_circles_demo(image):
    dst = cv2.pyrMeanShiftFiltering(image, 10, 100)
    c_image = cv2.cvtColor(dst, cv2.COLOR_RGB2GRAY)

    circles_detected = []

    try:
        circles = cv2.HoughCircles(c_image, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=7, maxRadius=35)
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            circles_detected.append([i[0], i[1], i[2]])
    except:
        pass

    # cv2.imshow("Circles", image)
    return circles_detected


def process_img(image, model, model_sign, index):
    threshold = 0.95

    img_bird_view = lane_finding_pipeline(image)

    img_bird_view = cv2.resize(img_bird_view, None, fx=0.4, fy=1, interpolation=cv2.INTER_CUBIC)

    detection_img = detect_warp(img_bird_view)
    detection_img = cv2.resize(detection_img, (200, 200))

    detect_img = np.asarray(detection_img)
    detect_img = cv2.resize(detect_img, (32, 32))

    detect_img = preProcessing(detect_img)
    detect_img = detect_img.reshape(1, 32, 32, 1)

    # Predict
    classIndex = int(model.predict_classes(detect_img))
    prediction = model.predict(detect_img)
    probVal = np.amax(prediction)

    if probVal > threshold:
        if classIndex != 6:
            # cv2.imwrite('C:/Users/GAO Xing/Desktop/RtMapsGit/Rtmaps_Autopilot_Project/AMI_Project/data/road_1_' + str(index) + '.png', detection_img)  # 写入图片
            index = index + 1

        if classIndex == 0:
            cv2.putText(detection_img, "^", (20, 100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)
        elif classIndex == 1:
            cv2.putText(detection_img, "<", (20, 100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)
        elif classIndex == 2:
            cv2.putText(detection_img, ">", (20, 100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)
        elif classIndex == 3:
            cv2.putText(detection_img, "< & ^", (20, 100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)
        elif classIndex == 4:
            cv2.putText(detection_img, "^ & >", (20, 100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)
        elif classIndex == 5:
            cv2.putText(detection_img, "STOP", (20, 100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)

    cv2.imshow('bird_view_img', img_bird_view)
    cv2.imshow('Road', detection_img)

    # Testing
    image_temp = detect_warp_sign(image, (0, 480), (0, 0), (640, 0), (640, 480))

    bl = [440, 250]
    tl = [440, 50]
    tr = [640, 50]
    br = [640, 250]
    warped = detect_warp_sign(image, bl, tl, tr, br)
    warped = cv2.resize(warped, (200, 200), interpolation=cv2.INTER_CUBIC)
    # cv2.imshow("Warped", warped)

    imgHSV = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
    hue_min = 0
    hue_max = 30
    sat_min = 40
    sat_max = 255
    val_min = 150
    val_max = 255

    lower = np.array([hue_min, sat_min, val_min])
    upper = np.array([hue_max, sat_max, val_max])
    mask = cv2.inRange(imgHSV, lower, upper)

    imgResult = cv2.bitwise_and(warped, warped, mask=mask)
    # cv2.imshow('Filter', imgResult)

    circles_detected = detect_circles_demo(imgResult)

    for circles in circles_detected:

        x = circles[0]
        y = circles[1]
        r = circles[2]

        bl = [x - r, y + r]
        tl = [x - r, y - r]
        tr = [x + r, y - r]
        br = [x + r, y + r]

        a_warp = detect_warp_sign(warped, bl, tl, tr, br)

        imgHSV = cv2.cvtColor(a_warp, cv2.COLOR_BGR2HSV)
        hue_min = 0
        hue_max = 180
        sat_min = 0
        sat_max = 255
        val_min = 240
        val_max = 255
        lower = np.array([hue_min, sat_min, val_min])
        upper = np.array([hue_max, sat_max, val_max])
        mask = cv2.inRange(imgHSV, lower, upper)

        content = cv2.bitwise_and(a_warp, a_warp, mask=mask)
        content = cv2.resize(content, (200, 200), interpolation=cv2.INTER_CUBIC)
        content = cv2.cvtColor(content, cv2.COLOR_BGR2GRAY)

        mask = add_mask(content, (100, 100), 100)
        output = cv2.add(content, np.zeros(np.shape(content), dtype=np.uint8), mask=mask)
        # index = index + 1

        detect_sign_img = np.asarray(output)
        detect_sign_img = cv2.resize(detect_sign_img, (32, 32))

        detect_sign_img = detect_sign_img.reshape(1, 32, 32, 1)

        # Predict
        classIndex = int(model_sign.predict_classes(detect_sign_img))
        prediction = model_sign.predict(detect_sign_img)
        probVal = np.amax(prediction)

        if probVal > threshold and 40 < y < 100:
            if classIndex < 3:
                cv2.rectangle(image_temp, (tl[0] + 440, tl[1] + 50), (br[0] + 440, br[1] + 50), (255, 255, 0), 3)
                # cv2.imwrite('C:/Users/GAO Xing/Desktop/RtMapsGit/Rtmaps_Autopilot_Project/AMI_Project/data/sign_51_' + str(classIndex) + '_' + str(index) + '.png', output)
                index = index + 1

            if classIndex == 0:
                cv2.putText(image_temp, "Speed limit : 30", (5, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
            elif classIndex == 1:
                cv2.putText(image_temp, "Speed limit : 60", (5, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
            elif classIndex == 2:
                cv2.putText(image_temp, "Speed limit : 90", (5, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('Detection', image_temp)

    cv2.waitKey(1)

    return index


def getContours(image_canny, image_warp):
    contours, hierarchy = cv2.findContours(image_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)

        # remove noise
        if area > 400:

            # true = closed
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.01 * peri, True)
            # nb corner
            objCor = len(approx)
            x, y, w, h = cv2.boundingRect(approx)
            if objCor == 3:
                objType = "Triangle"
            elif objCor == 4:
                aspectRatio = float(w) / float(h)

                if 0.95 < aspectRatio < 1.05:
                    objType = "Square"
                else:
                    objType = "Rectangle"

            elif objCor == 8:
                objType = "Octagon"
            elif 9 < objCor < 20:
                objType = "Circle"
            else:
                objType = "???"

            if objType == "Circle":
                # cv2.drawContours(image_warp, cnt, -1, (0, 255, 0), 2)
                # cv2.rectangle(image_warp, (x, y), (x + w, y + h), (255, 255, 0), 5)

                p_tl = (x, y)
                p_br = (x + w, y + h)

                warped = detect_warp_sign(image_warp, (x, y + h), (x, y), (x + w, y), (x + w, y + h))
                warped = cv2.resize(warped, (200, 200), interpolation=cv2.INTER_CUBIC)

                imgGray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
                mask = add_mask(imgGray, (100, 100), 100)
                output = cv2.add(imgGray, np.zeros(np.shape(imgGray), dtype=np.uint8), mask=mask)

                return output, image_warp, p_tl, p_br

    output = image_warp
    output = cv2.resize(output, (200, 200), interpolation=cv2.INTER_CUBIC)

    imgGray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
    mask = add_mask(imgGray, (100, 100), 100)
    output = cv2.add(imgGray, np.zeros(np.shape(imgGray), dtype=np.uint8), mask=mask)

    p_tl = (0, 0)
    p_br = (0, 0)

    return output, image_warp, p_tl, p_br


def detect_warp_sign(img, bl, tl, tr, br):
    img_size = (img.shape[1], img.shape[0])
    offset = 0

    src = np.float32([
        bl,  # bottom-left corner
        tl,  # top-left corner
        tr,  # top-right corner
        br  # bottom-right corner
    ])

    dst = np.float32([
        [offset, img_size[1]],  # bottom-left corner
        [offset, 0],  # top-left corner
        [img_size[0] - offset, 0],  # top-right corner
        [img_size[0] - offset, img_size[1]]  # bottom-right corner
    ])

    # Calculate the transformation matrix and it's inverse transformation
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, img_size)

    return warped


class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)

        self.model = None
        self.model_sign = None
        self.index = 0

    def Dynamic(self):
        self.add_input("in", rtmaps.types.ANY)
        self.add_output("out", rtmaps.types.AUTO)

    def Birth(self):
        # TODO Calibration of AMI's camera
        pickle_in = open("C:/Users/GAO Xing/Desktop/RtMapsGit/Rtmaps_Autopilot_Project-main/AMI_Project/AMI_Project/models/road_trained_5.p", "rb")
        self.model = pickle.load(pickle_in)
        pickle_in = open("C:/Users/GAO Xing/Desktop/RtMapsGit/Rtmaps_Autopilot_Project-main/AMI_Project/AMI_Project/models/sign_trained_s0.p", "rb")
        self.model_sign = pickle.load(pickle_in)
        self.index = 0

    def Core(self):
        image_raw_data = self.inputs["in"].ioelt.data  # create an ioelt from the input

        im_array = np.copy(np.frombuffer(image_raw_data, dtype=np.dtype("uint8")))
        im_array = np.reshape(im_array, (480, 640, 4))

        im_array_i3 = im_array[:, :, :3]
        im_array = im_array[:, :, :3][:, :, ::-1]

        self.index = process_img(im_array_i3, self.model, self.model_sign, self.index)

        tmpImage = rtmaps.types.IplImage()
        tmpImage.color_model = "RGB"
        tmpImage.channel_seq = "RGB"
        tmpImage.image_data = np.copy(im_array)

        self.outputs["out"].write(tmpImage)

    def Death(self):
        cv2.destroyAllWindows()
