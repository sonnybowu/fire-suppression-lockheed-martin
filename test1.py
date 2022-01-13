from student_base import student_base
import time
import numpy as np
import json
import math

class Fire:
	def __init__(self, lats, lons):
		self.lats = lats
		self.lons = lons
	
	@property
	def area(self):
		"""
		Returns fire area. Uses Shoelace Theorem. 
		"""
		return np.abs(
			np.dot( self.lons, np.roll(self.lats,1) ) - np.dot( self.lats,np.roll(self.lons,1) )
			) / 2
	
	@property
	def centroid(self):
		"""
		Returns the center coordinate of a given polygon. 
		"""
		return [np.mean(self.lats), np.mean(self.lons)]
	
	def water_needed(self, threshold):
		"""
		Returns amount of water (seconds) needed to spend over the fire to reduce it to a threshold area.
		"""
		return ( math.log(threshold) - math.log(self.area) ) / math.log(0.95)

	def distance_to(self, pos):
		"""
		Calculates distance from current drone position to centroid of fire.
		"""
		return np.linalg.norm(self.centroid, pos)

class my_flight_controller(student_base):
	"""
	Student flight controller class.
	Parameters
	----------
	student_base : student_base
		Class defining functionality enabling base functionality
	
	Methods
	-------
	student_run(self, telemetry: Dict, commands: Dict (optional))
		Method that takes in telemetry and issues drone commands.
	"""
	FIRE_MAP = 'maps/boston_fire.json'
	ALT = 100
	anchor = None
	fires = []		
	
	def get_lon_coords(self, path):
		"""
		Returns a list of the longitude coordinates of the fires from the map's json file.
		"""
		f = open(path)
		data = json.load(f)
		lon_coords = data['data_fs']['xs']
		return [_ for _ in lon_coords]
		
	def get_lat_coords(self, path):
		"""
		Returns a list of the latitude coordinates of the fires from the map's json file.
		"""
		f = open(path)
		data = json.load(f)
		lat_coords = data['data_fs']['ys']
		return [_ for _ in lat_coords]

	def add_fires(self):
		"""
		Adds all fires on map to the fires field.
		"""
		lats = self.get_lat_coords(self.FIRE_MAP)
		lons = self.get_lon_coords(self.FIRE_MAP)
		my_flight_controller.fires = [Fire(lats[i], lons[i]) for i in range(len(lats))]
	
	def closest_fire(self, pos):
		"""
		Returns Fire closest to pos.
		"""
		return min(my_flight_controller.fires, key=lambda f: f.distance_to(pos))
	
	def student_run(self, telemetry, commands):
		def move_to(pos, label):
			lat, lon = pos
			print(f'Get to {label}')
			self.goto(lat, lon, 100)
			err = np.linalg.norm([lat - telemetry['latitude'], lon - telemetry['longitude']])
			tol = 0.0001
			while err > tol:
				print(f'Aircraft enroute to {label}')
				time.sleep(10)
				err = np.linalg.norm([lat - telemetry['latitude'], lon - telemetry['longitude']])
			my_flight_controller.anchor = (lat, lon)
			print(f'Arrived at {label}')


		self.add_fires()
		print("Printing telemetry")
		for i in range(4):
			print(telemetry)
			time.sleep(0.5)
		
		print("Arming")
		self.arm()
		print("Taking off")
		home = (telemetry['latitude'], telemetry['longitude'])
		self.takeoff()
		
		print("Waiting 6 seconds")
		time.sleep(6)
		
		for _ in range(len(my_flight_controller.fires)):
			# Get water
			move_to((42.3608, -70.9904), 'water')
			time.sleep(6)

			# Put out fire
			next_fire = my_flight_controller.fires.pop( self.closest_fire(my_flight_controller.anchor) )
			move_to(next_fire.centroid, 'fire')
			time.sleep(6)

		move_to(home, 'base')
		time.sleep(10)
		print("Landing"), self.land()
		
		while telemetry['in_air']:
			time.sleep(0.1)
			
		print("Landed")
		while True:
			lat = telemetry['latitude']
			lon = telemetry['longitude']
			print(f'Latitude: {lat}, Longitude: {lon}')
			time.sleep(1)
		
if __name__ == "__main__":
	fcs = my_flight_controller()
	fcs.run()
