from .hardware_interface import HardwareInterface
from abc import ABC, abstractmethod

class LightingInterface(HardwareInterface):
    @abstractmethod
    def turn_on(self)->None:
        pass
    @abstractmethod
    def turn_off(self)->None:
        pass
    @abstractmethod
    def set_intensity(self, valuse: float)->None:
        pass