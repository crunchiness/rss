import numpy as np
import cv2
import rss_adam_utils as utils
import os
from matplotlib import pyplot as plt

path = '../resources/'
file = '0.jpg'

for file in os.listdir(path):
    print(path + file)
    utils.hsv_hist(path + file)