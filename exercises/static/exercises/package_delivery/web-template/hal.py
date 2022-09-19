import rospy
import cv2
import threading
import time
from datetime import datetime
from shared.magnet import Magnet

from drone_wrapper import DroneWrapper
from shared.image import SharedImage
from shared.value import SharedValue

# Hardware Abstraction Layer
class HAL:
    IMG_WIDTH = 320
    IMG_HEIGHT = 240
    
    def __init__(self):
        rospy.init_node("HAL")

        self.shared_frontal_image = SharedImage("halfrontalimage")
        self.shared_ventral_image = SharedImage("halventralimage")
        self.shared_x = SharedValue("x")
        self.shared_y = SharedValue("y")
        self.shared_z = SharedValue("z")
        self.shared_takeoff_z = SharedValue("sharedtakeoffz")
        self.shared_az = SharedValue("az")
        self.shared_azt = SharedValue("azt")
        self.shared_vx = SharedValue("vx")
        self.shared_vy = SharedValue("vy")
        self.shared_vz = SharedValue("vz")
        self.shared_landed_state = SharedValue("landedstate")
        self.shared_position = SharedValue("position",3)
        self.shared_velocity = SharedValue("velocity",3)
        self.shared_orientation = SharedValue("orientation",3)
        self.shared_yaw_rate = SharedValue("yawrate")
        self.shared_package_state = SharedValue("packagestate")

        self.shared_CMD =  SharedValue("CMD")

        self.image = None
        self.drone = DroneWrapper(name="rqt",ns="/iris/")
        self.magnet = Magnet()

        # Update thread
        self.thread = ThreadHAL(self.update_hal)

        

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
   

    # Function to start the update thread
    def start_thread(self):
        self.thread.start()
    
    # Get Image from ROS Driver Camera
    def get_frontal_image(self):
        image = self.drone.get_frontal_image()
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.shared_frontal_image.add(image_rgb)

    def get_ventral_image(self):
        image = self.drone.get_ventral_image()
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.shared_ventral_image.add(image_rgb)

    def get_position(self):
        pos = self.drone.get_position()
        self.shared_position.add(pos)

    def get_velocity(self):
        vel = self.drone.get_velocity()
        self.shared_velocity.add(vel )

    def get_yaw_rate(self):
        yaw_rate = self.drone.get_yaw_rate()
        self.shared_yaw_rate.add(yaw_rate)

    def get_orientation(self):
        orientation = self.drone.get_orientation()
        self.shared_orientation.add(orientation )

    def get_landed_state(self):
        state = self.drone.get_landed_state()
        self.shared_landed_state.add(state)

    def set_cmd_pos(self):
        x = self.shared_x.get()
        y = self.shared_y.get()
        z = self.shared_z.get()
        az = self.shared_az.get()

        self.drone.set_cmd_pos(x, y, z, az)

    def set_cmd_vel(self):
        vx = self.shared_vx.get()
        vy = self.shared_vy.get()
        vz = self.shared_vz.get()
        az = self.shared_azt.get()
        self.drone.set_cmd_vel(vx, vy, vz, az)

    def set_cmd_mix(self):
        vx = self.shared_vx.get()
        vy = self.shared_vy.get()
        z = self.shared_z.get()
        az = self.shared_azt.get()
        self.drone.set_cmd_mix(vx, vy, z, az)

    def takeoff(self):
        h = self.shared_takeoff_z.get()
        self.drone.takeoff(h)

    def land(self):
        self.drone.land()
    
    def set_cmd_pick(self):
        self.magnet.set_cmd_pick()

    def set_cmd_drop(self):
        self.magnet.set_cmd_drop()

    def get_pkg_state(self):
        state = self.magnet.get_pkg_state()
        self.shared_package_state.add(state)

    def update_hal(self):
        CMD = self.shared_CMD.get()

        self.get_frontal_image()
        self.get_ventral_image()
        self.get_position()
        self.get_velocity()
        self.get_yaw_rate()
        self.get_orientation()
        self.get_landed_state()
        self.get_pkg_state()
        
        if CMD == 0:  # POS
            self.set_cmd_pos()
        elif CMD == 1:  # VEL
            self.set_cmd_vel()
        elif CMD == 2:  # MIX
            self.set_cmd_mix()
        elif CMD == 3:  # TAKEOFF
            self.takeoff()
        elif CMD == 4:  # LAND
            self.land()
        elif CMD == 5:  # PKG PICK
            self.set_cmd_pick()
        elif CMD == 6:  # PKG PICK
            self.set_cmd_drop()

    # Destructor function to close all fds
    def __del__(self):
        self.shared_frontal_image.close()
        self.shared_ventral_image.close()
        self.shared_x.close()
        self.shared_y.close()
        self.shared_z.close()
        self.shared_takeoff_z.close()
        self.shared_az.close()
        self.shared_azt.close()
        self.shared_vx.close()
        self.shared_vy.close()
        self.shared_vz.close()
        self.shared_landed_state.close()
        self.shared_position.close()
        self.shared_velocity.close()
        self.shared_orientation.close()
        self.shared_yaw_rate.close()
        self.shared_package_state.close()

class ThreadHAL(threading.Thread):
    def __init__(self, update_function):
        super(ThreadHAL, self).__init__()
        self.time_cycle = 80
        self.update_function = update_function

    def run(self):
        while(True):
            start_time = datetime.now()

            self.update_function()

            finish_time = datetime.now()

            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0

            if(ms < self.time_cycle):
                time.sleep((self.time_cycle - ms) / 1000.0)