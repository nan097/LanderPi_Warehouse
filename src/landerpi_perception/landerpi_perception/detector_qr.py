from .detector_base import Detector


class QrDetector(Detector):
    def detect(self, image, target_class):
        raise NotImplementedError('qr backend not integrated')
