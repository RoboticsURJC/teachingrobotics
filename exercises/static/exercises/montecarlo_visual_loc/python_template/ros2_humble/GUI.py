import json
#import math
#import matplotlib.pyplot as plt
import threading
import cv2
import base64

from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI
from console_interfaces.general.console import start_console
from map import Map
from HAL import getPose3d

# Graphical User Interface Class

class GUI(MeasuringThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # Payload vars
        self.payload = {'image': '', 'map': '', 'particles': ''}
        self.init_coords = (171, 63)
        self.start_coords = (201, 85.5)
        self.map = Map(getPose3d)

        # Particles
        self.particles = []
        # Image
        self.image = None
        self.image_lock = threading.Lock()
        # User position
        #self.user_position = (0, 0)
        #self.user_angle = (0, 0)

        self.start()

    # Prepares and sends a map to the websocket server
    def update_gui(self):

        # Payload Image Message
        if self.image is not None:
            _, encoded_image = cv2.imencode(".JPEG", self.image)
            self.payload["image"] = base64.b64encode(encoded_image).decode("utf-8")

        # Payload Map Message
        pos_message = self.map.getRobotCoordinates()
        if pos_message == self.init_coords:
            pos_message = self.start_coords
        ang_message = self.map.getRobotAngle()
        pos_message = str(pos_message + ang_message)
        self.payload["map"] = pos_message

        # Payload User Message
        #pos_message_user = self.user_position
        #ang_message_user = self.user_angle
        #pos_message_user = pos_message_user + ang_message_user
        #pos_message_user = str(pos_message_user)
        #self.payload["user"] = pos_message_user

        # Payload Particles Message
        if self.particles:
            self.payload["particles"] = json.dumps(self.particles)
        else:
            self.payload["particles"] = json.dumps([])

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def showParticles(self, particles):
        if particles:
            self.particles = particles
            scale_y = 15
            offset_y = 63
            scale_x = -30
            offset_x = 171
            for particle in self.particles:
                particle[1] = scale_y * particle[1] + offset_y
                particle[0] = scale_x * particle[0] + offset_x
        else:
            self.particles = []
    
    # Function to set the next image to be sent
    def setImage(self, image):
        with self.image_lock:
            self.image = image

host = "ws://127.0.0.1:2303"
gui = GUI(host)

# Redirect the console
start_console()

# Expose to the user
def showImage(img):
    gui.setImage(img)

def showParticles(particles):
    gui.showParticles(particles)
