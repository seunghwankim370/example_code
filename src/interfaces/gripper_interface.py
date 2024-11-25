from .hardware_interface import HardwareInterface
from abc import ABC, abstractmethod

class GripperInterface(HardwareInterface):
    @abstractmethod
    def grip_go(self, pos )->None:
        pass

    
    # @abstractmethod
    # def grip(self)->None:
    #     pass
    # @abstractmethod
    # def release(self)->None:
    #     pass
    # @abstractmethod
    # def force(self, force: float)->None:
    #     pass