import rss_adam_utils as utils
import os
import sys

if len(sys.argv) == 2:
    utils.determine_boundaries(sys.argv[1])
else:
    path = '../resources/'

    for pic_file in os.listdir(path):
        if '.jpg' in pic_file or '.png' in pic_file:
            utils.determine_boundaries(path + pic_file)
