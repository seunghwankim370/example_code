from dataclasses import dataclass
from typing import Dict, List, Callable, Optional
from enum import Enum, auto
import time
import logging
from threading import Event

class SequenceStatus(Enum):
    IDLE = auto()
    RUNNING = auto()
    PAUSED = auto()
    ERROR = auto()
    COMPLETED = auto()

@dataclass
class SequenceCondition:            #       check=lambda: not device_manager.get_device("robot1").get_status()["is_moving"] -> check status 
    check: Callable[[], bool]       #       time out -> time out then transition 
    timeout: float = 10.0           #

@dataclass
class SequenceAction:
    name: str
    action: Callable
    conditions: List[SequenceCondition]
    next_action: Optional[str] = None
    on_error: Optional[str] = None

class SequenceBuilder:
    def __init__(self):
        self.actions: Dict[str, SequenceAction] = {}
        
    def add_action(self, name: str, action: Callable, 
                  conditions: List[SequenceCondition],
                  next_action: str = None,
                  on_error: str = None):
        self.actions[name] = SequenceAction(
            name=name,
            action=action,
            conditions=conditions,
            next_action=next_action,
            on_error=on_error
        )
        return self

    def build(self):
        return Sequence(self.actions)

class Sequence:
    def __init__(self, actions: Dict[str, SequenceAction]):
        self.actions = actions
        self.current_action = None
        
    def start(self, start_action: str):
        self.current_action = self.actions[start_action]
        self._execute_current()
        
    def _execute_current(self):
        if not self.current_action:
            return
            
        for condition in self.current_action.conditions:
            if not condition.check():
                return
                
        try:
            self.current_action.action()
            if self.current_action.next_action:
                self.current_action = self.actions[self.current_action.next_action]
        except Exception:
            if self.current_action.on_error:
                self.current_action = self.actions[self.current_action.on_error]