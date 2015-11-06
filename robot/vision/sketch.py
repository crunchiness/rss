import cv2
import numpy as np

path = '../resources/'

# img = utils.inrange_red(path + '60.jpg')
# img = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
# plt.imshow(img)
# plt.show()

train_data = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype = np.float32)
results_data = np.array([0, 1, 2], dtype = np.float32)
classifier = cv2.NormalBayesClassifier()
classifier.train(train_data, results_data)

predict_data = np.array([[0.83, 0.12, 0]], dtype = np.float32)
print classifier.predict(predict_data)

predict_data = np.array([[0.11, 0.89, 0]], dtype = np.float32)
print classifier.predict(predict_data)

predict_data = np.array([[0.11, 0.19, 1]], dtype = np.float32)
print classifier.predict(predict_data)
