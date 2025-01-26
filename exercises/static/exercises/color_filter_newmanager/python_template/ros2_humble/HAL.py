import rclpy

from jderobot_drones.drone_wrapper import DroneWrapper
from jderobot_drones.image_sub import ImageSubscriberNode

### HAL INIT ###

print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init()
    




