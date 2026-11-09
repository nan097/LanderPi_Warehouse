from .detector_base import Detector


class ApriltagDetector(Detector):
    def detect(self, image, target_class):
        raise NotImplementedError('apriltag backend not integrated')
