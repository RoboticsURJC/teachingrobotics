import rclpy
import threading
import time

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.camera import CameraNode

# Hardware Abstraction Layer
IMG_WIDTH = 320
IMG_HEIGHT = 240

freq = 30.0

print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=None)

    ### HAL INIT ###
    motor_node = MotorsNode("/cmd_vel", 4, 0.3)
    odometry_node = OdometryNode("/odom")
    camera_node = CameraNode("/camera/image_raw")

    # Spin nodes so that subscription callbacks load topic data
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(odometry_node)
    executor.add_node(camera_node)
    def __auto_spin() -> None:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0)
            time.sleep(1/freq)
    executor_thread = threading.Thread(target=__auto_spin, daemon=True)
    executor_thread.start()

# Pose
def getPose3d():
    return odometry_node.getPose3d()

# Image
def getImage():
    image = camera_node.getImage()
    while image == None:
        image = camera_node.getImage()
    return image.data

# Linear speed
def setV(v):
    motor_node.sendV(float(v))

# Angular speed
def setW(w):
    motor_node.sendW(float(w))
