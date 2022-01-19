from geopandas.tools.sjoin import _nearest_query
from student_base import student_base
import time
import numpy as np
import json
import math
import geopandas as gpd
from shapely.geometry import Point, LineString, Polygon
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
	#CONSTANTS
	FIRE_MAP = 'maps/boston_fire.json'
	ALT = 100
	
	#STORAGE
	fires = []	
	water = gpd.read_file('./data/waterbodies.geojson')['geometry']

	#LOCATIONS
	anchor = None
	home = None 	
	
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
	
	def closest_water(self, pos):
		"""
		Returns closest water to pos.
		"""
		pos = Point(*pos[::-1])
		points = self.water.apply(lambda x: nearest_points(x,pos)[0])
		closest_point = min(points, key=lambda x: pos.distance(x))
		print('closest',closest_point)
		return (closest_point.y, closest_point.x)

	def student_run(self, telemetry, commands):
		#~~~~~~~~~SETUP & MOTION~~~~~~~~~
		def setup():
			self.add_fires()
			print("Printing telemetry")
			
			for i in range(4):
				print(telemetry)
				time.sleep(0.5)
			
			print("Waiting 6 seconds")
			time.sleep(6)

			print("Arming")
			self.arm()
			print("Taking off")
			self.home = self.anchor = (telemetry['latitude'], telemetry['longitude'])
			#print('home', self.home)
			self.takeoff()

		def tear_down():
			move_to(self.home, 'base')
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
			self.anchor = (lat, lon)
			print(f'Arrived at {label}')


		#~~~~~~~~COMMAND SEQUENCE~~~~~~~~
		setup()
		for _ in range(len(self.fires)):
			move_to(self.closest_water(self.anchor), 'water')
			time.sleep(6)
			next_fire = self.closest_fire(self.anchor)
			self.fires.remove(next_fire)
			time.sleep(6)
			move_to(next_fire.centroid, 'fire')
			time.sleep(6)
		tear_down()

if __name__ == "__main__":
	fcs = my_flight_controller()
	fcs.run()

	# a = Point(-70.9899999, 42.3599999)
	# choose_point(a)

	# print(d)
	# print(
	# 	nearest_points(d,a)[0]
	# )
	
	# a = LineString([Point(-78.929596, 40.349718), Point(-78.934682, 40.360541), Point(-78.965688, 40.388129)])
	# b = Point(-78.96, 40.36)
	# print(
	# 	nearest_points(a,b)[0]
	# )
	# a = shapely.geometry.LineString([shapely.geometry.Point(0,0), shapely.geometry.Point(1,0)])
	# b = shapely.geometry.Point(0.5,1)

	# print(shapely.ops.nearest_points(a,b)[0]) # POINT (0.5, 0)
	
