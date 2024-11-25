from abc import ABC, abstractmethod
from typing import Protocol, Dict, Type

class HardwareInterface(ABC):
    @abstractmethod
    def initialize(self)->None:
        pass

    @abstractmethod
    def shutdown(self)->None:
        pass

    @abstractmethod
    def get_status(self)->dict:
        pass

    