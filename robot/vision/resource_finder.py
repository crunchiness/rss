import os
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
    def __init__(self, which, path=None):
        self.path = path
        self.which = which
        self.detector = cv2.ORB_create(nfeatures=5000)
        self.model = self.load_cube_model()
        self.matcher = self.init_matcher()

    def init_matcher(self):
        matcher = cv2.FlannBasedMatcher(flann_params, {})
        # matcher.add()
        return matcher

    def load_cube_model(self):
        if self.path is None:
            base_path = os.path.dirname(os.path.dirname(os.path.realpath('__file__'))) + '/'
            path = base_path + 'student/robot/vision/small_models/{}.png'.format(self.which)
        else:
            path = self.path
        model_img = cv2.imread(path)

        # maybe we're on DICE
        if model_img is None and self.path is None:
            base_path = os.path.dirname(os.path.dirname(os.path.realpath('__file__'))) + '/'
            path = base_path + 'robot/vision/small_models/{}.png'.format(self.which)
            model_img = cv2.imread(path)

        assert model_img is not None, 'failed to load model ' + path
        features = self.detect_features(model_img)
        return {
            'keypoints': features[0],
            'descriptors': features[1]
        }

    def detect_features(self, frame):
        """wrapper for detector.detectAndCompute"""
        keypoints, descriptors = self.detector.detectAndCompute(frame, None)
        if descriptors is None:
            descriptors = []
        return keypoints, descriptors

    def detect_cube(self, frame):
        frame_points, frame_descrs = self.detect_features(frame)

        # no points -> no cubes
        if len(frame_points) < MIN_MATCH_COUNT:
            return None

        matches = self.matcher.knnMatch(frame_descrs, self.model['descriptors'], k=2)

        # Lowe's ratio test
        matches = [m[0] for m in matches if len(m) == 2 and m[0].distance < m[1].distance * 0.9]

        # no matching points -> no matching cubes
        if len(matches) < MIN_MATCH_COUNT:
            return None

        model_points = [self.model['keypoints'][m.trainIdx].pt for m in matches]
        real_points = [frame_points[m.queryIdx].pt for m in matches]
        model_points, real_points = np.float32((model_points, real_points))
        H, status = cv2.findHomography(model_points, real_points, cv2.RANSAC, 5.0)
        if status is None:
            return None

        status = status.ravel() != 0
        if status.sum() < MIN_MATCH_COUNT:
            return None

        model_points, real_points = model_points[status], real_points[status]

        return {
            'p1': real_points,
            'H': H
        }


class DetectionConfirmer:
    def __init__(self):
        self.min_number = 15
        self.results = []

    def add(self, res):
        # TODO: could somehow incorporate the shape of the points detected
        self.results.append(len(res) > self.min_number)

    def get_result(self):
        trues = sum([1 if i else 0 for i in self.results])
        falses = sum([0 if i else 1 for i in self.results])
        self.results = []
        return trues > falses


def get_mean(detection):
    return tuple(np.mean(detection['p1'], 0))
