import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import thread

from robot.vision.resource_finder_test import MockIO

io = MockIO()

thresholds = {
    'lower_hue': 0,
    'upper_hue': 179,
    'lower_sat': 0,
    'upper_sat': 255,
    'lower_val': 0,
    'upper_val': 255
}

def video(a, b):
    def make_mask(image):
        if thresholds['lower_hue'] > thresholds['upper_hue']:
            lower_bound1 = np.array([thresholds['lower_hue'], thresholds['lower_sat'], thresholds['lower_val']], np.uint8)
            upper_bound1 = np.array([179, thresholds['upper_sat'], thresholds['upper_val']], np.uint8)
            lower_bound2 = np.array([0, thresholds['lower_sat'], thresholds['lower_val']], np.uint8)
            upper_bound2 = np.array([thresholds['upper_hue'], thresholds['upper_sat'], thresholds['upper_val']], np.uint8)
            mask1 = cv2.inRange(image, lower_bound1, upper_bound1)
            mask2 = cv2.inRange(image, lower_bound2, upper_bound2)
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            lower_bound = np.array([thresholds['lower_hue'], thresholds['lower_sat'], thresholds['lower_val']], np.uint8)
            upper_bound = np.array([thresholds['upper_hue'], thresholds['upper_sat'], thresholds['upper_val']], np.uint8)
            mask = cv2.inRange(image, lower_bound, upper_bound)
        return mask

    def scale_down(image):
        rows = image.shape[0]
        cols = image.shape[1]
        matrix = cv2.getRotationMatrix2D((0, 0), 0, 0.5)
        return cv2.warpAffine(image, matrix, (cols / 2, rows / 2))

    while True:
        io.cameraGrab()
        img = io.cameraRead()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        range_mask = make_mask(hsv)
        masked_hsv = cv2.bitwise_and(hsv, hsv, mask=range_mask)
        masked_bgr = cv2.cvtColor(masked_hsv, cv2.COLOR_HSV2BGR)
        display_img = np.zeros((600, 800, 3), np.uint8)
        display_img[0:300, 0:400, :] = scale_down(img)
        display_img[0:300, 400:800, :] = scale_down(masked_bgr)
        display_img[300:600, 0:400, 0] = scale_down(range_mask)
        display_img[300:600, 0:400, 1] = scale_down(range_mask)
        display_img[300:600, 0:400, 2] = scale_down(range_mask)
        io.imshow('Window', display_img)


thread.start_new_thread(video, ('Thread-1', 2,))

axcolor = 'lightgoldenrodyellow'
axlh = plt.axes([0.1, 0.9, 0.65, 0.03], axisbg=axcolor)
axuh = plt.axes([0.1, 0.8, 0.65, 0.03], axisbg=axcolor)
axls = plt.axes([0.1, 0.7, 0.65, 0.03], axisbg=axcolor)
axus = plt.axes([0.1, 0.6, 0.65, 0.03], axisbg=axcolor)
axlv = plt.axes([0.1, 0.5, 0.65, 0.03], axisbg=axcolor)
axuv = plt.axes([0.1, 0.4, 0.65, 0.03], axisbg=axcolor)

slh = Slider(axlh, 'lower_hue', 0, 179, valinit=0)
suh = Slider(axuh, 'upper_hue', 0, 179, valinit=179)
sls = Slider(axls, 'lower_sat', 0, 255, valinit=0)
sus = Slider(axus, 'upper_sat', 0, 255, valinit=255)
slv = Slider(axlv, 'lower_val', 0, 255, valinit=0)
suv = Slider(axuv, 'upper_val', 0, 255, valinit=255)

def update(val):
    thresholds['lower_hue'] = slh.val
    thresholds['upper_hue'] = suh.val
    thresholds['lower_sat'] = sls.val
    thresholds['upper_sat'] = sus.val
    thresholds['lower_val'] = slv.val
    thresholds['upper_val'] = suv.val
    print thresholds

slh.on_changed(update)
suh.on_changed(update)
sls.on_changed(update)
sus.on_changed(update)
slv.on_changed(update)
suv.on_changed(update)

plt.show()
