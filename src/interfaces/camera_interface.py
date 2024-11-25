from .hardware_interface import HardwareInterface
from abc import ABC, abstractmethod

class CameraInterface(HardwareInterface):
    @abstractmethod
    def capture(self)->bytes:
        pass

    @abstractmethod
    def set_exposure_time(self, value: float)->None:
        pass

    @abstractmethod
    def set_gain(self, value: float)-> None:
        pass

    @abstractmethod
    def set_gamma(self, value: float)-> None:
        pass

    @abstractmethod
    def set_offsetx(self, value: float)-> None:
        pass

    @abstractmethod
    def set_offsety(self, value: float)-> None:
        pass

    @abstractmethod
    def set_height(self, value: float)-> None:
        pass

    @abstractmethod
    def set_width(self, value: float)-> None:
        pass