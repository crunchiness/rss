import rss_adam_utils as utils
import os

path = '../resources/'
pic_file = '0.jpg'

for pic_file in os.listdir(path):
    print(path + pic_file)
    utils.hsv_hist(path + pic_file)
