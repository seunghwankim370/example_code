from threading import Thread, Event
from queue import Queue
from typing import Dict, Any
import time
from hardware_factory import HardwareFactory
import logging as logger

class DeviceThread:
    def __init__(self, name: str, device: Any, event_queue: Queue):
        self.name = name
        self.device = device
        self.event_queue = event_queue
        self.stop_event = Event()
        self.thread = Thread(target=self._run, name=name, daemon = False)
        
    def _run(self):
        while not self.stop_event.is_set():
            try:
                self.device.process()  
                time.sleep(0.1)  
                
            except Exception as e:
                self.event_queue.put({
                    "type": "error",
                    "device": self.name,
                    "error": str(e)
                })
    
    def start(self):
        self.thread.start()
        
    def stop(self):
        self.stop_event.set()
        self.thread.join()

class DeviceManager:
    def __init__(self):
        self._devices = {}
        self._device_threads: Dict[str, DeviceThread] = {}
        self._event_queue = Queue()
        self._event_handlers = {}
        self._camera_model_pairs = {}
        self._factory = HardwareFactory()

        self._event_thread = Thread(target=self._process_events)
        self._event_thread.daemon = True  
        self._event_thread.start()
    
    def register_device(self, name: str, category: str, model: str, **config):
        try:

            device = self._factory.create_hardware(category, model, **config)
            device.initialize()
            self._devices[name] = device
            

            thread = DeviceThread(name, device, self._event_queue)
            self._device_threads[name] = thread
            thread.start()
            
            print(f"Device {name} registered and started")
            
        except Exception as e:
            print(f"Failed to register device {name}: {e}")
            raise
    
    def _process_events(self):
        while True:
            try:
                event = self._event_queue.get()
                event_type = event["type"]
                
                if event_type == "error":
                    self._handle_error(event)
                elif event_type in self._event_handlers:
                    for handler in self._event_handlers[event_type]:
                        try:
                            handler(event)
                        except Exception as e:
                            print(f"Error in event handler: {e}")
                            
            except Exception as e:
                print(f"Error in event processing: {e}")
    
    def _handle_error(self, event):
        device_name = event["device"]
        error_msg = event["error"]
        print(f"Error in device {device_name}: {error_msg}")
    
    def register_event_handler(self, event_type: str, handler):
        if event_type not in self._event_handlers:
            self._event_handlers[event_type] = []
        self._event_handlers[event_type].append(handler)
    
    def get_device(self, name: str) -> Any:
        return self._devices.get(name)
    
    def shutdown_all_devices(self):
        print("Shutting down all devices...")
        
        for name, thread in self._device_threads.items():
            print(f"Stopping thread for device {name}")
            thread.stop()
        
        for name, device in self._devices.items():
            print(f"Shutting down device {name}")
            device.shutdown()
        
        self._devices.clear()
        self._device_threads.clear()
        print("All devices shut down")

    def register_camera_device_pair(self, camera_name: str, device_name: str):

        if camera_name not in self._devices:
            raise ValueError(f"Camera {camera_name} not registered")
            
        if device_name not in self._devices:
            raise ValueError(f"Device {device_name} not registered")
            
        camera = self._devices[camera_name]
        device = self._devices[device_name]
        device.set_image_queue(camera.image_queue)
        
        self._camera_model_pairs[camera_name] = device_name
        print(f"Paired camera {camera_name} with device {device_name}")

