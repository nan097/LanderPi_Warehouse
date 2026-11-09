from .detector_base import Detector


class YoloDetector(Detector):
    def detect(self, image, target_class):
        raise NotImplementedError('yolo backend not integrated')
