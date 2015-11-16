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
        self.matcher = cv2.FlannBasedMatcher(flann_params, {})
        self.zoidberg = self.load_zoidberg()
        self.matcher.add([self.zoidberg['descriptors']])

    def load_zoidberg(self):
        image = cv2.imread('images/zoidberg_model_small.png')
        # finds 1000 key points and descriptors
        keypoints, descrs = self.detector.detectAndCompute(image, None)
        return {
            'keypoints': keypoints,
            'descriptors': descrs
        }

    def detect_features(self, frame):
        """wrapper for detector.detectAndCompute"""
        keypoints, descriptors = self.detector.detectAndCompute(frame, None)
        if descriptors is None:
            descriptors = []
        return keypoints, descriptors

    def find_possible_rects(self):
        """
        somehow detect possible places where the cubes can be in images, return bounding boxes
        :return:
        """
        # TODO
        # no matching points -> no matching cubes
        return [[0, 0, 799, 799]]

    def detect_cube(self, frame):
        frame_points, frame_descrs = self.detect_features(frame)

        # no points -> no cubes
        if len(frame_points) < MIN_MATCH_COUNT:
            return []

        # find 2 matches for every point, save one with smaller distance
        # TODO: check what the distance actually means, and what properties does a match have
        matches = []
        for m in self.matcher.knnMatch(frame_descrs, self.zoidberg['descriptors'], k=2):
            if len(m) == 2:
                x = m[0] if m[0].distance < m[1].distance else m[1]
                matches.append(x)
            elif len(m) == 1:
                matches.append(m[0])
        # print len(matches), 'matches'
        # no matching points -> no matching cubes
        if len(matches) < MIN_MATCH_COUNT:
            return []

        cubes = []
        for rect in self.find_possible_rects():

            # for match in matches:
            #     print 'q', match.queryIdx, 't', match.trainIdx

            model_points = [self.zoidberg['keypoints'][m.trainIdx].pt for m in matches]
            real_points = [frame_points[m.queryIdx].pt for m in matches]

            model_points, real_points = np.float32((model_points, real_points))

            H, status = cv2.findHomography(model_points, real_points, cv2.RANSAC, 3.0)
            if status is None:
                return []

            status = status.ravel() != 0
            if status.sum() < MIN_MATCH_COUNT:
                return []

            model_points, real_points = model_points[status], real_points[status]

            x0, y0, x1, y1 = rect
            quad = np.float32([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
            quad = cv2.perspectiveTransform(quad.reshape(1, -1, 2), H).reshape(-1, 2)

            track = {
                'target': self.zoidberg,
                'p0': model_points,
                'p1': real_points,
                'H': H,
                'quad': quad
            }
            cubes.append(track)

        cubes.sort(key=lambda t: len(t['p0']), reverse=True)
        return cubes


a = CubeDetector()

cap = cv2.VideoCapture(2)
while True:
    ret, frame = cap.read()
    tracked = a.detect_cube(frame)
    print 'detected {} cubes'.format(len(tracked))

    for tr in tracked:
        cv2.polylines(frame, [np.int32(tr['quad'])], True, (0, 0, 255), 2)
        for (x, y) in np.int32(tr['p1']):
            cv2.circle(frame, (x, y), 2, (0, 255, 0))

    cv2.imshow('plane', frame)
    cv2.waitKey(20)


# frame = cv2.imread('images/watching.png')
# frame = cv2.imread('images/zoidberg.png')
# frame = cv2.imread('images/zoidberg_test_upside_down.png')

