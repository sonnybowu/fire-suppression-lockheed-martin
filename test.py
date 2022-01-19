from student_base import student_base
import time
import numpy as np
import json
import math
import geopandas as gpd
from shapely.geometry import Point
from shapely.ops import nearest_points

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
		drone_x, drone_y = pos[0], pos[1]
		fire_x, fire_y = self.centroid[0], self.centroid[1]
		dist = math.sqrt((drone_x - fire_x)**2 + (drone_y - fire_y)**2)
		return dist

class Tour():
	def __init__(self, start):
		self.start = start
		self.water = gpd.read_file('./data/waterbodies.geojson')['geometry']
		self.fires = self._load_fires()
	
	def _load_fires(self):
		f = open('maps/boston_fire.json')
		data = json.load(f)
		lats = data['data_fs']['ys']
		lons = data['data_fs']['xs']
		return [Fire(lats[i], lons[i]) for i in range(len(lats))]

	def closest_fire(self, pos):
		"""
		Returns Fire closest to pos.
		"""
		return min(self.fires, key=lambda f: f.distance_to(pos))
	
	def closest_water(self, pos):
		"""
		Returns water closest to pos.
		"""
		pos = Point(*pos[::-1])
		points = self.water.apply(lambda x: nearest_points(x,pos)[0])
		closest_point = min(points, key=lambda x: pos.distance(x))
		return (closest_point.y, closest_point.x)

	def gen_tour(self):
		"""
		Generates a nearest neighbors tour through all fires on map.
		"""
		traversal = []
		pos = self.start
		for _ in range(len(self.fires)):
			pos = self.closest_water(pos)
			traversal.append((pos, 'water'))
			next_fire = self.closest_fire(pos)
			self.fires.remove(next_fire)
			pos = next_fire.centroid
			traversal.append((pos, 'fire'))
		return traversal

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
	start = None
	def student_run(self, telemetry, commands):			
		#~~~~~~~~~SETUP & MOTION~~~~~~~~~
		def setup():
			print("Printing telemetry")
			time.sleep(1)
			print(telemetry)
			
			print("Waiting 3 seconds")
			time.sleep(3)

			print("Arming")
			self.arm()
			print("Taking off")
			self.start = (telemetry['latitude'], telemetry['longitude'])
			self.takeoff()

		def execute_tour(tour):
			for pt in tour:
				lat, lon = pt[0]
				print(f'Get to {pt[1]}')
				self.goto(lat, lon, 100)
				tol = 0.0001
				err = np.linalg.norm([lat - telemetry['latitude'], lon - telemetry['longitude']])
				while err > tol:
					print(f'Aircraft enroute to {pt[1]}')
					time.sleep(10)
					err = np.linalg.norm([lat - telemetry['latitude'], lon - telemetry['longitude']])
				print(f'Arrived at {pt[1]}')
				time.sleep(10)

		#~~~~~~~~COMMAND SEQUENCE~~~~~~~~
		setup()
		execute_tour(Tour(self.start).gen_tour())
		
		print("Landing"), self.land()
		while telemetry['in_air']:
			time.sleep(0.1)		
		print("Landed")

if __name__ == "__main__":
	fcs = my_flight_controller()
	fcs.run()
	
