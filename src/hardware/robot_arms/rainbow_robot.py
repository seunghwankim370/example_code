from dataclasses import dataclass
from interfaces import RobotInterface
from threading import Lock
import logging
from typing import Optional, Tuple

@dataclass
class URRobotConfig:
    ip: str
    port: int =                                         30002
    acceleration: float =                               1.0
    velocity: float =                                   1.0
    home_position: Tuple[float, float, float] =         (0.0, 0.0, 0.0)

class URRobot(RobotInterface):
    def __init__(self, config: URRobotConfig):
        self._config =                                  config
        self._logger =                                  logging.getLogger(__name__)
        self._lock =                                    Lock()
        
        self._is_connected =                            False
        self._current_position =                        (0.0, 0.0, 0.0)
        self._is_moving =                               False
        self._target_position =                         None
        
    def initialize(self) -> None:
        try:
            self._logger.info(f"Connecting to UR robot at {self._config.ip}")
            # 실제로는 여기서 로봇 연결
            self._is_connected = True
            self._logger.info("Robot initialized successfully")
        except Exception as e:
            self._logger.error(f"Failed to initialize robot: {str(e)}")
            raise

    def process(self):
        if not self._is_connected or not self._is_moving:
            return
            
        try:
            if self._target_position:
                self._current_position = self._target_position
                self._is_moving = False
                self._target_position = None
                self._logger.info(f"Moved to position: {self._current_position}")
                
        except Exception as e:
            self._logger.error(f"Error during movement: {str(e)}")
            self._is_moving = False

    def move_to(self, position: Tuple[float, float, float], speed: float = 1.0) -> None:
        with self._lock:
            if not self._is_connected:
                raise RuntimeError("Robot is not initialized")
                
            try:
                self._target_position = position
                self._is_moving = True
                self._logger.info(f"Moving to position: {position} with speed: {speed}")
            except Exception as e:
                self._logger.error(f"Failed to start movement: {str(e)}")
                raise

    def home(self) -> None:
        self.move_to(self._config.home_position)

    def get_position(self) -> Tuple[float, float, float]:

        return self._current_position

    def get_status(self) -> dict:
        return {
            "connected":                                self._is_connected,
            "is_moving":                                self._is_moving,
            "current_position":                         self._current_position,
            "target_position":                          self._target_position
        }

    def shutdown(self) -> None:
        try:
            if self._is_connected:
                self._is_connected = False
                self._logger.info("Robot shutdown completed")
        except Exception as e:
            self._logger.error(f"Error during shutdown: {str(e)}")
            raise