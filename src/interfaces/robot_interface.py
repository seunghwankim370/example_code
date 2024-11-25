from .hardware_interface import HardwareInterface
from abc import ABC, abstractmethod
from typing import Tuple, List

class RobotInterface(HardwareInterface):
    @abstractmethod
    def move_to(self, position: Tuple[float, float, float], speed: float = 1.0)->None:
        pass
    @abstractmethod
    def home(self) -> None:
        pass
    @abstractmethod
    def get_position(self)-> Tuple[float, float, float]:
        pass