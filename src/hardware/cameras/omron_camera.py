from ...interfaces.camera_interface import CameraInterface
from dataclasses import dataclass
import logging
from queue import Queue
from multiprocessing import Queue as ProcessQueue
import stapipy as st
from threading import Lock
from typing import Optional
import numpy as np

@dataclass
class OmronCameraConfig:
    ip: str
    exposure_time: float =              5000
    width: float =                      1920
    height: float =                     1080
    gain: float =                       1.0
    gamma: float =                      1.0
    offset_x: float =                   0
    offset_y: float =                   0

class OmronCamera(CameraInterface):
    def __init__(self, config: OmronCameraConfig):
        self._config =                                  config
        self._logger =                                  logging.geLogger(__name__)

        self.image_queue =                              Queue(maxsize = 10)
        self.process_queue =                            ProcessQueue(maxsize = 10)

        self._st_system =                               None
        self._st_device =                               None
        self._st_datastream =                           None
        self._remote_nodemap =                          None
        self._is_grabbing =                             False

        self._lock =                                    Lock()

        self._logger.info(f"OmronCamera instance created with config: {self._config}")

    def process(self):

        if not self._is_grabbing:
            return
        
        try:
            with self._st_datastream.retrieve_buffer() as buffer:
                

                if not buffer.info.is_image_present:
                    image = buffer.data.copy()
                
                    if not self.image_queue.full():
                        self.image_queue.put(image)
                    if not self.process_queue.full():
                        self.process_queue.put(image)

                

        except Exception as e:
            self._logger.error(f"Error in image acquisition:{str(e)}")

    def initialize(self) -> None:
        try:
            self._logger.info("Initializing Omron camera...")
            
            self._st_system = st.create_system()
            
            self._st_system.update_device_list()
            devices = self._st_system.get_device_list()
            target_device = None
            
            for device in devices:
                nodemap = device.remote_port.nodemap
                if nodemap.get_node('GevDeviceIPAddress').value == self._config.ip:
                    target_device = device
                    break
                    
            if not target_device:
                raise RuntimeError(f"Camera with IP {self._config.ip} not found")
            
            self._st_device = target_device.create_device()
            self._remote_nodemap = self._st_device.remote_port.nodemap
            
            self._st_datastream = self._st_device.create_datastream()
            
            self._configure_camera()
            
            self._logger.info("Omron camera initialized successfully")
            
        except Exception as e:
            self._logger.error(f"Failed to initialize camera: {str(e)}")
            self.shutdown()
            raise


    def _configure_camera(self):
        with self._lock:
            try:
                self._remote_nodemap.get_node('ExposureTime').value =               self._config.exposure_time
                self._remote_nodemap.get_node('Gain').value =                       self._config.gain
                self._remote_nodemap.get_node('Gamma').value =                      self._config.gamma
                
                self._remote_nodemap.get_node('Width').value =                      self._config.width
                self._remote_nodemap.get_node('Height').value =                     self._config.height
                self._remote_nodemap.get_node('OffsetX').value =                    self._config.offset_x
                self._remote_nodemap.get_node('OffsetY').value =                    self._config.offset_y
                
                self._remote_nodemap.get_node('GevSCPSPacketSize').value =          self._config.packet_size
                
                self._logger.info("Camera configuration completed")
                
            except Exception as e:
                self._logger.error(f"Failed to configure camera: {str(e)}")
                raise

    def get_config(self) -> OmronCameraConfig:
        return self._config

    def update_config(self, new_config: OmronCameraConfig):
        with self._lock:
            try:
                was_grabbing = self._is_grabbing
                if was_grabbing:
                    self.stop_acquisition()
                
                self._config = new_config
                self._configure_camera()
                
                if was_grabbing:
                    self.start_acquisition()
                    
                self._logger.info("Camera configuration updated")
                
            except Exception as e:
                self._logger.error(f"Failed to update configuration: {str(e)}")
                raise

    def get_image_for_processing(self) -> Optional[np.ndarray]:
        if not self._process_queue.empty():
            return self._process_queue.get()
        return None