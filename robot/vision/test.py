import os

from robot.vision import vision

path = '../../resources/'

for pic_file in os.listdir(path):
    if '.jpg' in pic_file or '.png' in pic_file:
        vision.detect_pieces(path + pic_file, save=True)
