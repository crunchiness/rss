import cv2
import numpy as np

MIN_MATCH_COUNT = 10
FLANN_INDEX_LSH = 6
flann_params = {
    'algorithm': FLANN_INDEX_LSH,
    'table_number': 6,
    'key_size': 12,
    'multi_probe_level': 1
}


class CubeDetector:
    def __init__(self):
        self.detector = cv2.ORB_create(nfeatures=1000)
        print 'loading', 'self.models'
        self.models = self.load_cube_models()
        print 'loading', 'self.matchers'
        self.matchers = self.init_matchers()

    def init_matchers(self):
        matchers = {}
        for key in self.models.keys():
            matcher = cv2.FlannBasedMatcher(flann_params, {})
            matcher.add([self.models[key]['descriptors']])
            matchers[key] = matcher
        return matchers

    def load_cube_models(self):
        model_imgs = [
            cv2.imread('small_models/zoidberg.png'),
            cv2.imread('small_models/mario.png'),
            cv2.imread('small_models/wario.png'),
            cv2.imread('small_models/watching.png')
        ]
        features = map(lambda x: self.detector.detectAndCompute(x, None), model_imgs)
        models = {
            'zoidberg': {'keypoints': features[0][0], 'descriptors': features[0][1]},
            'mario':    {'keypoints': features[1][0], 'descriptors': features[1][1]},
            'wario':    {'keypoints': features[2][0], 'descriptors': features[2][1]},
            'watching': {'keypoints': features[3][0], 'descriptors': features[3][1]}
        }
        return models

    def detect_features(self, frame):
        """wrapper for detector.detectAndCompute"""
        keypoints, descriptors = self.detector.detectAndCompute(frame, None)
        if descriptors is None:
            descriptors = []
        return keypoints, descriptors

    def detect_zoidberg(self, frame):
        return self.detect_cube(frame, 'zoidberg')

    def detect_cube(self, frame, key):
        frame_points, frame_descrs = self.detect_features(frame)

        # no points -> no cubes
        if len(frame_points) < MIN_MATCH_COUNT:
            return None

        # find 2 matches for every point, save one with smaller distance
        matches = []
        for m in self.matchers[key].knnMatch(frame_descrs, self.models[key]['descriptors'], k=2):
            if len(m) == 2:
                x = m[0] if m[0].distance < m[1].distance else m[1]
                matches.append(x)
            elif len(m) == 1:
                matches.append(m[0])

        # no matching points -> no matching cubes
        if len(matches) < MIN_MATCH_COUNT:
            return None

        model_points = [self.models[key]['keypoints'][m.trainIdx].pt for m in matches]
        real_points = [frame_points[m.queryIdx].pt for m in matches]

        model_points, real_points = np.float32((model_points, real_points))

        H, status = cv2.findHomography(model_points, real_points, cv2.RANSAC, 3.0)

        if status is None:
            return None

        status = status.ravel() != 0
        if status.sum() < MIN_MATCH_COUNT:
            return None

        model_points, real_points = model_points[status], real_points[status]

        x0, y0, x1, y1 = [0, 0, 799, 799]  # TODO image size
        quad = np.float32([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
        quad = cv2.perspectiveTransform(quad.reshape(1, -1, 2), H).reshape(-1, 2)

        return {
            'target': self.models[key],
            'p0': model_points,
            'p1': real_points,
            'H': H,
            'quad': quad
        }

def get_mean(detection):
    return tuple(np.mean(detection['p1'], 0))

a = CubeDetector()

cap = cv2.VideoCapture(1)
while True:
    ret, frame = cap.read()
    detection = a.detect_zoidberg(frame)
    if detection is None:
        continue
    cv2.circle(frame, get_mean(detection), 3, (255, 0, 0))
    for (x, y) in np.int32(detection['p1']):
        cv2.circle(frame, (x, y), 2, (0, 255, 0))

    cv2.imshow('plane', frame)
    cv2.waitKey(20)


# frame = cv2.imread('images/watching.png')
# frame = cv2.imread('images/zoidberg.png')
# frame = cv2.imread('images/zoidberg_test_upside_down.png')

