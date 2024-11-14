import json
import math
import numpy as np

class Map:
	def __init__(self, lidar_object):
		# Define the object used for
		# websocket communication
		self.payload = {}

		self.lidar_topic = lidar_object
        
    # Get the JSON data as string
	def get_json_data(self):
		self.payload["lasers"], self.payload["ranges"] = self.setLaserValues()

		message = json.dumps(self.payload)
		return message
    	
	def RTx(self, angle, tx, ty, tz):
		RT = np.matrix([[1, 0, 0, tx], [0, math.cos(angle), -math.sin(angle), ty], [0, math.sin(angle), math.cos(angle), tz], [0,0,0,1]])
		return RT
        
	def RTy(self, angle, tx, ty, tz):
		RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty], [-math.sin(angle), 0, math.cos(angle), tz], [0,0,0,1]])
		return RT
    
	def RTz(self, angle, tx, ty, tz):
		RT = np.matrix([[math.cos(angle), -math.sin(angle), 0, tx], [math.sin(angle), math.cos(angle),0, ty], [0, 0, 1, tz], [0,0,0,1]])
		return RT  	
        
	# Interpret the Laser values
	def setLaserValues(self):
		# Init laser array
		self.lidar = self.lidar_topic()
		return self.lidar