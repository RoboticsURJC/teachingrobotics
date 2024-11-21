import numpy as np
import math
from parse_configuration import Config
from math import pi as pi
from hal import HAL

flag = 0

class Map:
	def __init__(self,laser_object, pose3d):
		self.config = Config()
		self.hal = HAL()
		self.pose3d = pose3d
		self.laser_topic = laser_object

	def RTx(self, angle, tx, ty, tz):
		RT = np.matrix([[1, 0, 0, tx], [0, math.cos(angle), -math.sin(angle), ty], 
						[0, math.sin(angle), math.cos(angle), tz], [0, 0, 0, 1]])
		return RT
		
	def RTy(self, angle, tx, ty, tz):
		RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty],
						[-math.sin(angle), 0, math.cos(angle), tz], [0, 0, 0, 1]])
		return RT
		
	def RTz(self, angle, tx, ty, tz):
		RT = np.matrix([[math.cos(angle), -math.sin(angle), 0, tx], [math.sin(angle), math.cos(angle), 0, ty],
						[0, 0, 1, tz], [0, 0, 0, 1]])
		return RT
		
	def RTVacuum(self):
		RTz = self.RTz(pi/2, 50, 70, 0)
		return RTz

	def getRobotAngle(self):
		pose = self.pose3d.getPose3d()
		rt = pose.yaw

		ty = math.cos(-rt) - math.sin(-rt)
		tx = math.sin(-rt) + math.cos(-rt)

		return tx, ty

	# Function to reset
	def reset(self):
		# Nothing to do, service takes care!
		pass
	
	def setRobotValues(self):
		global flag
		pose = self.pose3d.getPose3d()
		yaw = pose.yaw
		x = pose.x
		y = pose.y
		if(flag == 0):
			self.initial_x = self.pose3d.getPose3d().x
			self.initial_y = self.pose3d.getPose3d().y
			self.initial_yaw = self.pose3d.getPose3d().yaw
			flag = 1
		x_desired = self.config.pos_x
		y_desired = self.config.pos_y
		yaw_desired = self.config.orientation
		radius = 0.15
		base = radius/2
		coord_contorno = []
		coord_robot = []
		# Vertice 1
		alfa = math.asin((base)/radius)
		xr, yr = self.local2relative(0, 0, radius, math.pi - alfa)
		xg, yg = self.relative2global(x_desired + (x - self.initial_x), y_desired + (y - self.initial_y), xr, yr, yaw_desired + (yaw - self.initial_yaw))
		xc, yc = self.global2canvas(xg, yg)
		coord_contorno.append((0,0))
		coord_contorno [0] = (xc, yc)
	 	# Vertice 2
		xr, yr = self.local2relative(0, 0, radius, math.pi + alfa)
		xg, yg = self.relative2global(x_desired + (x - self.initial_x), y_desired + (y - self.initial_y), xr, yr, yaw_desired + (yaw - self.initial_yaw))
		xc, yc = self.global2canvas(xg, yg)
		coord_contorno.append((0,0))
		coord_contorno [1] = (xc, yc)
		# Vertice 3
		xr, yr = self.local2relative(0, 0, radius, 0)
		xg, yg = self.relative2global(x_desired + (x - self.initial_x), y_desired + (y - self.initial_y), xr, yr, yaw_desired + (yaw - self.initial_yaw))
		xc, yc = self.global2canvas(xg,yg)
		coord_contorno.append((0,0))
		coord_contorno [2] = (xc, yc)

	 	# Position of the robot in the canvas
		xc, yc = self.global2canvas(x_desired + (x - self.initial_x), y_desired + (y - self.initial_y))
		coord_robot.append((0,0))
		coord_robot = (xc, yc)
		return coord_robot, coord_contorno
	
	def setSonarValues(self):
    try:
        # Initialize arrays
        self.sonares = []
        self.dist = []

        # Define the number of sonars and cone angle
        num_sonars = 8
        cone = 0.261799  # ~15 degrees in radians

        # Process sonar configurations and distances
        for i in range(num_sonars):
            # Get sonar configuration
            sonar_config = getattr(self.config, f"sonar_{i}")
            self.sonares.append((sonar_config['POS_X'], sonar_config['POS_Y'], sonar_config['ORIENTATION']))

            # Get sonar distance data
            sonar_data = getattr(self.hal, f"sonar_{i}").getSonarData()
            distance = sonar_data.distances

            # Handle edge cases for infinite or negative infinite distances
            if distance == float("inf"):
                distance = sonar_data.maxRange
            elif distance == float("-inf"):
                distance = sonar_data.minRange

            self.dist.append(distance)

        # Get pose data
        pose = self.pose3d.getPose3d()
        x_desired = self.config.pos_x
        y_desired = self.config.pos_y
        yaw_desired = self.config.orientation
        yaw = pose.yaw
        x = pose.x
        y = pose.y

        # Initialize outputs
        vertices = []
        sonar_sensor = []

        # Process each sonar
        for i in range(num_sonars):
            distance = self.dist[i]
            hypotenuse = distance / math.cos(cone / 2)

            # Compute local-to-relative triangle vertices based on orientation
            orientation = self.sonares[i][2]
            orientations = [orientation - cone / 2, orientation + cone / 2] if orientation < 0 else [orientation + cone / 2, orientation - cone / 2]
            xr_1, yr_1 = self.local2relative(self.sonares[i][0], self.sonares[i][1], hypotenuse, orientations[0])
            xr_2, yr_2 = self.local2relative(self.sonares[i][0], self.sonares[i][1], hypotenuse, orientations[1])

            # Convert relative to global coordinates
            xg_1, yg_1 = self.relative2global(x_desired + (x - self.initial_x), y_desired + (y - self.initial_y), xr_1, yr_1, yaw_desired + (yaw - self.initial_yaw))
            xg_2, yg_2 = self.relative2global(x_desired + (x - self.initial_x), y_desired + (y - self.initial_y), xr_2, yr_2, yaw_desired + (yaw - self.initial_yaw))

            # Convert global to canvas coordinates
            xc_1, yc_1 = self.global2canvas(xg_1, yg_1)
            xc_2, yc_2 = self.global2canvas(xg_2, yg_2)

            vertices.append((xc_1, yc_1, xc_2, yc_2))

            # Compute sonar sensor position in canvas coordinates
            xg, yg = self.relative2global(x_desired + (x - self.initial_x), y_desired + (y - self.initial_y), self.sonares[i][0], self.sonares[i][1], yaw_desired + (yaw - self.initial_yaw))
            xc, yc = self.global2canvas(xg, yg)
            sonar_sensor.append((xc, yc))

        return vertices, sonar_sensor

    except Exception as e:
        print(f"Error in setSonarValues: {e}")
        return [[0, 0], [0, 0]]

	def setLaserValues(self):
		try:
			pose = self.pose3d.getPose3d()
			yaw = pose.yaw
			x = pose.x
			y = pose.y
			x_desired = self.config.pos_x
			y_desired = self.config.pos_y
			yaw_desired = self.config.orientation
			xg, yg = self.relative2global(x_desired + (x - self.initial_x), y_desired + (y - self.initial_y), self.config.laser['POS_X'], self.config.laser['POS_Y'], yaw_desired + (yaw - self.initial_yaw))
			xc, yc = self.global2canvas(xg, yg)
			laser_sensor = [xc, yc]
			laser = self.hal.laser.getLaserData()
			angle_min = laser.minAngle
			angle_max = laser.maxAngle
			angle_increment = laser.angleIncrement
			rays = np.arange(angle_min, angle_max + angle_increment, angle_increment)
			cont_rays = len(rays)
			laser_beam = []
			for i in range(0,cont_rays-1):
				laser_beam.append((0,0))
				if(i == 0):
					angle = -math.pi
				else:
					angle = angle + 0.015708
				dist = laser.values[i]
				if(dist == float("inf")):
					dist = laser.maxRange
				xr, yr = self.local2relative(self.config.laser['POS_X'], self.config.laser['POS_Y'], dist, angle)
				xg, yg = self.relative2global(x_desired + (x - self.initial_x), y_desired + (y - self.initial_y), xr, yr, yaw_desired + (yaw - self.initial_yaw))
				xc, yc = self.global2canvas(xg, yg)
				laser_beam[i] = (xc, yc)
			return laser_beam, laser_sensor
		except:
			return [[0, 0], [0, 0]]

	def local2relative(self, posx, posy, distance, orientation):
		xr = posx + math.cos(orientation)*distance
		yr = posy + math.sin(orientation)*distance
	
		return xr, yr
	
	def relative2global(self, posx, posy, xr, yr, yaw):
		xg = posx + math.cos(yaw)*xr - math.sin(yaw)*yr
		yg = posy + math.sin(yaw)*xr + math.cos(yaw)*yr
		
		return xg, yg
	
	def global2canvas(self, posx, posy):
		scale_y = 33.25; offset_y = 0.33
		scale_x = 33.25; offset_x = 0.33
		x = scale_x * posx + offset_x
		y = 729 - (scale_y * posy + offset_y)
	
		return x, y
