"""HSV contour detector; thresholds must be supplied for the actual lighting."""
from .detector_base import Detection, Detector


class ColorDetector(Detector):
    def __init__(self, class_name, hsv_lower, hsv_upper, min_area_pixels):
        import cv2
        import numpy as np
        self.cv2, self.np = cv2, np
        if min_area_pixels <= 0:
            raise ValueError('minimum area must be positive')
        self.class_name = class_name
        self.lower = np.array(hsv_lower, dtype=np.uint8)
        self.upper = np.array(hsv_upper, dtype=np.uint8)
        self.min_area = min_area_pixels

    def detect(self, image, target_class):
        if target_class != self.class_name:
            return []
        hsv = self.cv2.cvtColor(image, self.cv2.COLOR_BGR2HSV)
        mask = self.cv2.inRange(hsv, self.lower, self.upper)
        contours, _ = self.cv2.findContours(mask, self.cv2.RETR_EXTERNAL,
                                           self.cv2.CHAIN_APPROX_SIMPLE)
        results = []
        for contour in contours:
            area = self.cv2.contourArea(contour)
            if area < self.min_area:
                continue
            x, y, w, h = self.cv2.boundingRect(contour)
            # Heuristic mask fill ratio, not a calibrated classification probability.
            results.append(Detection(self.class_name, area / (w * h),
                                     x + w / 2, y + h / 2))
        return results
