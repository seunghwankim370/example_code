from core import (
   DeviceManager,
   HardwareFactory,
   SequenceBuilder,
   SequenceCondition,
   SequenceStatus
)
import random
import time
import logging
from typing import List, Tuple

def generate_random_positions(num_positions: int = 5) -> List[Tuple[float, float, float]]:
   positions = []
   for _ in range(num_positions):
       x = random.uniform(100, 500)    # RANDON NUMBER# RANDON NUMBER# RANDON NUMBER# RANDON NUMBER# RANDON NUMBER# RANDON NUMBER
       y = random.uniform(-300, 300)   # RANDON NUMBER# RANDON NUMBER# RANDON NUMBER# RANDON NUMBER# RANDON NUMBER# RANDON NUMBER
       z = random.uniform(0, 200)      # RANDON NUMBER# RANDON NUMBER# RANDON NUMBER# RANDON NUMBER# RANDON NUMBER# RANDON NUMBER
       positions.append((x, y, z))     # RANDON NUMBER# RANDON NUMBER# RANDON NUMBER# RANDON NUMBER# RANDON NUMBER# RANDON NUMBER
   return positions

def main():
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)

    factory = HardwareFactory()
    device_manager = DeviceManager()

    try:
        device_manager.register_device(
            name="camera1",
            category="camera",
            model="omron",
            ip="192.168.1.100",
            exposure_time=5000
        )

        device_manager.register_device(
            name="robot1",
            category="robot",
            model="rainbow",
            ip="192.168.1.101",
            home_position=(250, 0, 100)
        )

        device_manager.register_device(
            name="yolov6",
            category="engine",
            model="yolov6",
            model_path="",
            conf_thres=0.5
        )

        device_manager.register_device(
            name="imagenet",
            category="engine",
            model="imagenet",
            model_path="",
            conf_thres=0.5
        )

        device_manager.register_camera_device_pair("camera1", "yolov6")
        device_manager.register_camera_device_pair("camera1", "imagenet")

        builder = SequenceBuilder()
        positions = generate_random_positions(5)
        logger.info(f"Generated positions: {positions}")

        robot_ready = SequenceCondition(
            check=lambda: not device_manager.get_device("robot1").get_status()["is_moving"],
            timeout=10.0
        )

        camera_ready = SequenceCondition(
            check=lambda: device_manager.get_device("camera1").get_status()["is_ready"],
            timeout=5.0
        )

        SequenceBuilder.add_action(
            name="move_to_home",
            action=lambda: device_manager.get_device("robot1").home(),
            conditions=[robot_ready],
            next_action="move_to_pos_0"
        )

        for i, position in enumerate(positions):
            move_action = f"move_to_pos_{i}"
            capture_action = f"capture_pos_{i}"
            next_move = f"move_to_pos_{i+1}" if i < len(positions)-1 else None
            
            SequenceBuilder.add_action(
                name=move_action,
                action=lambda pos=position: device_manager.get_device("robot1").move_to(pos),
                conditions=[robot_ready],
                next_action=capture_action
            )

            SequenceBuilder.add_action(
                name=capture_action,
                action=lambda: device_manager.get_device("camera1").capture(),
                conditions=[robot_ready, camera_ready],
                next_action=next_move
            )

        sequence = SequenceBuilder.build()
        sequence.start("move_to_home")

        while True:                                                                            # Thread, Thread, Thread, Thread, Thread, Thread, Thread
            if sequence.status == SequenceStatus.COMPLETED:
                logger.info("Sequence completed, starting new sequence...")                   # Thread, Thread, Thread, Thread, Thread, Thread, Thread
                sequence.start("move_to_home")
            elif sequence.status == SequenceStatus.ERROR:
                logger.error("Sequence failed, restarting...")                                   # Thread, Thread, Thread, Thread, Thread, Thread, Thread
                sequence.start("move_to_home")
            time.sleep(1)                                                                    # Thread, Thread, Thread, Thread, Thread, Thread, Thread

    except Exception as e:
        logger.error(f"Error occurred: {e}")
    finally:
        device_manager.shutdown_all_devices()

if __name__ == "__main__":
    main()