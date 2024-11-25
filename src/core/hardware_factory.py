from typing import Dict, Type, Any
from ..interfaces.hardware_interface import HardwareInterface
import importlib
import os

class HardwareFactory:
    _hardware_registry: Dict[str, Dict[str, Type[HardwareInterface]]] = {
        "camera" : {},
        "gripper" : {},
        "lighting" : {},
        "robot" : {}
    }

    @classmethod
    def register_hardware(cls, category: str, model: str, hardware_class: Type[HardwareInterface]) -> None:
        if category not in cls._hardware_registry:
            cls._hardware_registry[category] = {}
        cls._hardware_registry[category][model] = hardware_class
    
    @classmethod
    def create_hardware(cls, category: str, model: str, **kwargs) -> HardwareInterface:
        if category not in cls._hardware_registry:
            raise ValueError(f"Unknown hardware category: {category}")
        
        if model not in cls._hardware_registry[category]:
            try:
                module = importlib.import_module(f"hardware.{category}.{model}")
                hardware_class = getattr(module, f"{model.capitalize()}")
                cls.register_hardware(category, model, hardware_class)
            except (ImportError, AttributeError) as e:
                raise ValueError(f"Unknown hardware model: {model} in category {category}")
        
        return cls._hardware_registry[category][model](**kwargs)
    
