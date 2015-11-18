"""Main vision class"""
from robot.vision.localisation import detect_pieces
from robot.vision.resource_finder import DetectionConfirmer, CubeDetector


class Vision:
    def __init__(self, io):
        self.io = io
        self.io.cameraSetResolution('high')
        self.belief = []
        self.resources = {}
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
        for i in range(0, 5):
            self.io.cameraGrab()
        img = self.io.cameraRead()

        assert img is not None, 'failed to read image'

        self.belief = detect_pieces(img, save=True, display=False)

        # Resource identification
        for i in range(0, 10):
            self.io.cameraGrab()
            img = self.io.cameraRead()
            for model_name in self.model_names:
                detection = self.cube_detectors[model_name].detect_cube(img)
                if detection is not None:
                    self.detection_confirmers[model_name].add(detection['p1'])

        for model_name in self.model_names:
            self.resources[model_name] = self.detection_confirmers[model_name].get_result()
