import numpy as np
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider
import matplotlib.gridspec as gridspec
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

def determine_boundaries(file):
    hsv_map = np.zeros((180, 256, 3), np.uint8)
    h, s = np.indices(hsv_map.shape[:2])
    hsv_map[:,:,0] = h
    hsv_map[:,:,1] = s
    hsv_map[:,:,2] = 255
    hsv_map = cv2.cvtColor(hsv_map, cv2.COLOR_HSV2BGR)

    bgr = cv2.imread(file)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h = cv2.calcHist([hsv], [0, 1], None, [180, 256], [0, 180, 0, 256])
    hist_scale = 10
    h1 = np.clip(h*0.005*hist_scale, 0, 1)
    hist_bgr = hsv_map*h1[:,:,np.newaxis] / 255.0

    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    hist_rgb = cv2.cvtColor(hist_bgr, cv2.COLOR_BGR2RGB)

    fig = plt.figure(figsize = (8,2))

    gs = gridspec.GridSpec(8, 2,
                           width_ratios=[1,1],
                           height_ratios=[15,30,1,1,1,1,1,1]
                           )

    a = fig.add_subplot(gs[0])
    hist_axes = plt.imshow(hist_rgb)
    plt.xlabel('saturation')
    plt.ylabel('hue')

    a = fig.add_subplot(gs[1])
    masked_hist_axes = plt.imshow(hist_rgb)
    plt.xlabel('saturation')
    plt.ylabel('hue')

    a = fig.add_subplot(gs[2])
    image_axes = plt.imshow(rgb)

    a = fig.add_subplot(gs[3])
    masked_image_axes = plt.imshow(rgb)

    axcolor = 'lightgoldenrodyellow'

    axlh = plt.subplot(gs[2, :], axisbg=axcolor)
    axuh = plt.subplot(gs[3, :], axisbg=axcolor)
    axls = plt.subplot(gs[4, :], axisbg=axcolor)
    axus = plt.subplot(gs[5, :], axisbg=axcolor)
    axlv = plt.subplot(gs[6, :], axisbg=axcolor)
    axuv = plt.subplot(gs[7, :], axisbg=axcolor)

    slh = Slider(axlh, 'lower_hue', 0, 179, valinit=0  , dragging=False, valfmt="%d")
    suh = Slider(axuh, 'upper_hue', 0, 179, valinit=179, dragging=False, valfmt="%d")
    sls = Slider(axls, 'lower_sat', 0, 255, valinit=0  , dragging=False, valfmt="%d")
    sus = Slider(axus, 'upper_sat', 0, 255, valinit=255, dragging=False, valfmt="%d")
    slv = Slider(axlv, 'lower_val', 0, 255, valinit=0  , dragging=False, valfmt="%d")
    suv = Slider(axuv, 'upper_val', 0, 255, valinit=255, dragging=False, valfmt="%d")

    def update(val):
        lh = slh.val
        uh = suh.val
        ls = sls.val
        us = sus.val
        lv = slv.val
        uv = suv.val

        lower_color_bound = np.array([lh, ls, lv], np.uint8)
        upper_color_bound = np.array([uh, us, uv], np.uint8)
        range_mask = cv2.inRange(hsv, lower_color_bound, upper_color_bound)
        masked_hsv = cv2.bitwise_and(hsv, hsv, mask = range_mask)
        masked_rgb = cv2.cvtColor(masked_hsv, cv2.COLOR_HSV2RGB)

        h = cv2.calcHist([masked_hsv], [0, 1], None, [180, 256], [0, 180, 0, 256])
        hist_scale = 10
        h1 = np.clip(h*0.005*hist_scale, 0, 1)
        masked_hist_bgr = hsv_map*h1[:,:,np.newaxis] / 255.0
        masked_hist_rgb = cv2.cvtColor(masked_hist_bgr, cv2.COLOR_BGR2RGB)

        masked_image_axes.set_data(masked_rgb)
        masked_hist_axes.set_data(masked_hist_rgb)

        fig.canvas.draw_idle()

    slh.on_changed(update)
    suh.on_changed(update)
    sls.on_changed(update)
    sus.on_changed(update)
    slv.on_changed(update)
    suv.on_changed(update)

    # mng = plt.get_current_fig_manager()
    # mng.full_screen_toggle()

    plt.show()