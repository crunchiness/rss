import numpy as np
from matplotlib import pyplot as plt
import cv2


def hsv_hist(file):
    hsv_map = np.zeros((180, 256, 3), np.uint8)
    h, s = np.indices(hsv_map.shape[:2])
    hsv_map[:, :, 0] = h
    hsv_map[:, :, 1] = s
    hsv_map[:, :, 2] = 255
    hsv_map = cv2.cvtColor(hsv_map, cv2.COLOR_HSV2BGR)

    bgr = cv2.imread(file)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    dark = hsv[..., 2] < 32
    hsv[dark] = 0
    h = cv2.calcHist([hsv], [0, 1], None, [180, 256], [0, 180, 0, 256])

    hist_scale = 10
    h1 = np.clip(h * 0.005 * hist_scale, 0, 1)
    vis_bgr = hsv_map * h1[:, :, np.newaxis] / 255.0

    fig = plt.figure()

    fig.add_subplot(1, 2, 1)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    plt.imshow(rgb)

    fig.add_subplot(1, 2, 2)
    vis_rgb = cv2.cvtColor(vis_bgr, cv2.COLOR_BGR2RGB)
    plt.imshow(vis_rgb)
    plt.xlabel('saturation')
    plt.ylabel('hue')

    plt.show()
