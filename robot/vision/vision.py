"""Main vision class"""
from robot.vision.localisation import detect_pieces
import numpy as np
import cv2
from robot.vision.resource_finder import DetectionConfirmer, CubeDetector, get_mean


class Vision:
    def __init__(self, io):
        self.io = io
        self.io.cameraSetResolution('high')
        self.belief = []
        self.model_names = [
            'zoidberg',
            'mario',
            'wario',
            'watching'
        ]
        self.detection_confirmers = {
            'zoidberg': DetectionConfirmer(),
            'mario': DetectionConfirmer(),
            'wario': DetectionConfirmer(),
            'watching': DetectionConfirmer()
        }
        self.cube_detectors = {
            'zoidberg': CubeDetector('zoidberg'),
            'mario': CubeDetector('mario'),
            'wario': CubeDetector('wario'),
            'watching': CubeDetector('watching')
        }

    def do_image(self):

        # Pieces identification
        for i in range(0, 1):
            self.io.cameraGrab()
        img = self.io.cameraRead()
        assert img is not None, 'failed to read image'
        # self.belief = detect_pieces(img, save=True, display=False)
        print self.belief

    def see_resources(self, model_name):
        self.model_names = [
            model_name
        ]
        resources = {}
        # img = None
        for i in range(0, 5):
            self.io.cameraGrab()
        img = self.io.cameraRead()
        for model_name in self.model_names:
            print 'Checking', model_name
            detection = self.cube_detectors[model_name].detect_cube(img)
            if detection:
                print len(detection['p1'])
                # for (x, y) in np.int32(detection['p1']):
                #     cv2.circle(img, (x, y), 2, (0, 255, 255))
                #     cv2.circle(img, get_mean(detection), 2, (255, 0, 0), 10)
                #     self.io.imshow('Window', img)
                resources[model_name] = {'mean': get_mean(detection), 'found': len(detection['p1']) > 20}
        # for model_name in self.model_names:
        #     resources[model_name] = self.detection_confirmers[model_name].get_result()

        # if detection_temp:
        #     # print resources
        #     for (x, y) in np.int32(detection_temp['p1']):
        #         cv2.circle(img, (x, y), 2, (0, 255, 255))
        #     if 'mario' in resources and resources['mario']:
        #         cv2.circle(img, get_mean(detection_temp), 2, (255, 0, 0), 10)
        #     cv2.imshow('Window', img)
        #     cv2.waitKey(10)
        return resources

