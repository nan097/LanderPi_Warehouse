"""Detector contract; image acquisition and TF belong to the perception server."""
from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass(frozen=True)
class Detection:
    class_name: str
    confidence: float
    center_u: float
    center_v: float


class Detector(ABC):
    @abstractmethod
    def detect(self, image, target_class):
        """Return a list of Detection instances for one image."""
        raise NotImplementedError
