import rss_adam_utils as utils
import os

path = '../resources/'

for pic_file in os.listdir(path):
    if '.jpg' in pic_file:
        utils.determine_boundaries(path + pic_file)
