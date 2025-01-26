import json
import cv2
import base64
import threading
import time

from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI
from src.manager.ram_logging.log_manager import LogManager
from console import start_console
import numpy as np


# Graphical User Interface Class

class GUI(MeasuringThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # Execution control vars
        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()
        self.host = host
        self.payload = {"image": ""}
        self.frame_rgb = None

        self.start()
        
    # Process incoming messages to the GUI
    def gui_in_thread(self, ws, message):
        # In this case
        if "pick" in message:
            base64_buffer = message[4:]

            if base64_buffer.startswith('data:image/jpeg;base64,'):
                base64_buffer = base64_buffer[len('data:image/jpeg;base64,'):]

            # Decodificar la cadena base64 a bytes
            image_data = base64.b64decode(base64_buffer)
   
            # Convertir los bytes a un array de numpy
            nparr = np.frombuffer(image_data, np.uint8)
    
            # Decodificar la imagen (convertirla a formato OpenCV)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            self.frame_rgb = img
            ack = {
                "id": "ack",  # Usamos 'id' como "ack" para identificar este mensaje
                "command": "update",
                "data": {
                    "message": "Ack recibido correctamente"
                }
            }

            self.send_to_client(json.dumps(ack))


    # Prepares and sends a map to the websocket server
    def update_gui(self):

        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)  
       
        message = json.dumps(self.payload)
        self.send_to_client(message)


    # Function to prepare image payload
    # Encodes the image as a JSON string and sends through the WS
    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image_to_be_shown = self.image_to_be_shown

        image = image_to_be_shown
        payload = {'image': '', 'shape': ''}

        if not image_to_be_shown_updated:
            return payload

        shape = image.shape
        
        frame = cv2.imencode('.JPEG', image)[1]
        encoded_image = base64.b64encode(frame)

        payload['image'] = encoded_image.decode('utf-8')
        payload['shape'] = shape

        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload
    
    # Function for student to call
    def showImage(self, image):
        with self.image_show_lock:
            self.image_to_be_shown = image
            self.image_to_be_shown_updated = True



    def getImage(self):
        if (self.frame_rgb is None):
            # TEMPORAL PARA PROBAR EL ENVIO DE IMAGENES   
            frame_rgb = np.ones((240, 320, 3), dtype="uint8") * 255  # Blanco

            return frame_rgb
        else:
            return self.frame_rgb
            
host = "ws://127.0.0.1:2303"
gui = GUI(host)

# Redirect the console
start_console()


# Expose the user functions
def showImage(image):
    gui.showImage(image)


def getImage():
    return gui.getImage()
