import cv2
from matplotlib import pyplot as plt

i = cv2.imread('1.jpg')

i = cv2.cvtColor( i, cv2.COLOR_RGB2HSV)

hue_hist = cv2.calcHist([i], [0], None, [180], [0, 180])
sat_hist = cv2.calcHist([i], [1], None, [256], [0, 256])

plt.plot(hue_hist)
plt.plot(sat_hist)
plt.show()