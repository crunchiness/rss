"""Main vision class"""

import numpy as np
import datetime
import cv2  # 3.0.0-dev
import time


THRESHOLDS = {
    'dark_green': {
        'h': (59, 82),
        's': (40, 255),
        'v': (0, 255)
    },
    'light_green': {
        'h': (33, 50),
        's': (101, 255),
        'v': (0, 255)
    },
    'orange': {
        'h': (5, 15),
        's': (160, 255),
        'v': (160, 255)
    },
    'blue': {
        'h': (95, 117),
        's': (100, 255),
        'v': (0, 255)
    },
    'white': {
        'h': (16, 40),
        's': (0, 36),
        'v': (153, 255)
    },
    'yellow': {
        'h': (21, 27),
        's': (102, 255),
        'v': (140, 255)
    },
    'black': {
        'h': (0, 255),
        's': (0, 255),
        'v': (0, 90)
    },
    'red': {
        'h': (173, 5),
        's': (100, 255),
        'v': (120, 255)
    }
}

MIN_PIXELS = 16 * 16  # minimum number of pixels needed to confirm presence of object

def detect_pieces(img_file, display=True, save=True):
    """
    detects shape (based on color)
    :return: list of tuples (bool, color)
    """
    bgr = cv2.imread(img_file) if type(img_file) == str else img_file

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    masks = [(threshold_image(hsv, THRESHOLDS[key]), key) for key in THRESHOLDS.keys()]
    belief = [(interpret_mask(mask), key) for mask, key in masks]
    belief_dict = {
        'black': False,
        'blue': False,
        'green': False,
        'red': False,
        'orange': False,
        'white': False,
        'yellow': False
    }

    for mask, key in masks:
        key = 'green' if key == 'light_green' or key == 'dark_green' else key
        belief_dict[key] |= interpret_mask(mask)

    if display or save:
        masked_images = [(mask_image(hsv, mask), key) for mask, key in masks]
        img = np.zeros((600, 800, 3), np.uint8)
        img.fill(255)
        padding = 40
        for row in xrange(2):
            for col in xrange(4):
                index = row * 4 + col
                masked_image, color = masked_images[index]
                top = row * (120 + padding)
                left = col * (160 + padding)
                img[top:top + 120, left: left + 160] = cv2.cvtColor(masked_image, cv2.COLOR_RGB2BGR)
                text = '{0} - {1}'.format(color, belief[index][0])
                cv2.putText(img, text, (left, top + 140), cv2.FONT_HERSHEY_PLAIN, color=(0, 0, 0), fontScale=1)
        img[440:440 + 120, 320:320 + 160] = bgr
        cv2.putText(img, 'original', (320, 440+140), cv2.FONT_HERSHEY_PLAIN, color=(0, 0, 0), fontScale=1)
        if save:
            cv2.imwrite('detection-' + datetime.datetime.now().isoformat() + '.png', img)
        if display:
            cv2.imshow('window', img)
            cv2.waitKey(500)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

    return belief_dict

def interpret_mask(mask):
    detected_pixels = len(np.nonzero(mask)[0])
    return detected_pixels > MIN_PIXELS

def mask_image(image_hsv, mask):
    """
    :param image_hsv:
    :param mask:
    :return: mask image RGB
    """
    masked_hsv = cv2.bitwise_and(image_hsv, image_hsv, mask=mask)
    return cv2.cvtColor(masked_hsv, cv2.COLOR_HSV2RGB)

def threshold_image(img_hsv, thresholds):
    h = thresholds['h']
    s = thresholds['s']
    v = thresholds['v']

    # if hue loops around
    if h[0] > h[1]:
        h1 = (h[0], 180)
        h2 = (0, h[1])
        threshold_min_1 = np.array([h1[0], s[0], v[0]], np.uint8)
        threshold_max_1 = np.array([h1[1], s[1], v[1]], np.uint8)
        threshold_min_2 = np.array([h2[0], s[0], v[0]], np.uint8)
        threshold_max_2 = np.array([h2[1], s[1], v[1]], np.uint8)
        frame_threshed_1 = cv2.inRange(img_hsv, threshold_min_1, threshold_max_1)
        frame_threshed_2 = cv2.inRange(img_hsv, threshold_min_2, threshold_max_2)
        frame_threshed = cv2.bitwise_or(frame_threshed_1, frame_threshed_2)
    else:
        threshold_min = np.array([h[0], s[0], v[0]], np.uint8)
        threshold_max = np.array([h[1], s[1], v[1]], np.uint8)
        frame_threshed = cv2.inRange(img_hsv, threshold_min, threshold_max)

    kernel = np.ones((2, 2), np.uint8)
    frame_threshed = cv2.erode(frame_threshed, kernel, iterations=2)
    frame_threshed = cv2.dilate(frame_threshed, kernel, iterations=2)

    return frame_threshed


class Vision:
    def __init__(self, io):
        self.io = io
        self.io.cameraSetResolution('low')
        self.belief = []

    def do_image(self):
        for i in range(0, 5):
            self.io.cameraGrab()
        img = self.io.cameraRead()
        if img.__class__ == np.ndarray:
            self.belief = detect_pieces(img, save=True, display=False)
            # cv2.imwrite('camera-' + datetime.datetime.now().isoformat() + '.png', img)
            time.sleep(0.5)
