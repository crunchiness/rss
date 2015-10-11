import rss_adam_utils as utils
import os

path = '../resources/'

for file in os.listdir(path):
	if '.jpg' in file:
		utils.determine_boundaries(path + file)