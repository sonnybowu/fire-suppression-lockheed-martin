from re import T
from student_base import student_base
import time
import numpy as np
import json
import math
import geopandas as gpd
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import nearest_points
import random

class Waypoint():
	def __init__(self, coords, time, target):
		self.coords = coords
		self.time = time
		self.target = target

class Fire(Waypoint):
	def __init__(self, poly):
		self.poly = poly
		super().__init__(poly.centroid, self.time(), self.target_area())
	
	def time(self):
		transit = 3
		if self.poly.area >= self.target_area(): transit += math.log(self.target_area() / self.poly.area) / math.log(.95) / 5.189
		return transit
	
	def target_area(self):
		return 1e-8

	def value(self):
		return round( (self.poly.area - self.poly.area * math.pow(.95, self.time*5.189))*1e10 )

class Water(Waypoint):
	def __init__(self, coords, time, target):
		super().__init__(coords, time, target)

class GPS():
	def __init__(self):
		self.water = gpd.read_file('./data/waterbodies.geojson')['geometry']
		self.fires = self._load_fires()

	def _load_fires(self):
		f = open('maps/Fire_Competition.json')
		data = json.load(f)
		shapes = zip(data['data_fs']['xs'], data['data_fs']['ys'])
		return [Fire(Polygon([*zip(*shape)])) for shape in shapes]

	def travel_time(self, p1, p2):
		return p1.distance(p2) / 1.1976737e-4

	def nearest_fire(self, pos):
		"""
		Returns Fire closest to pos.
		"""
		return min(self.fires, key=lambda f: pos.distance(f.coords))
	
	def nearest_water_pt(self, pos):
		"""
		Returns water closest to pos.
		"""
		points = self.water.apply(lambda x: nearest_points(x,pos)[0])
		return min(points, key=lambda x: pos.distance(x))
		
class Tour():
	def __init__(self, *args):
		self.start = Point(-70.6185, 42.98575)#Point(-70.9897, 42.3594)
		self.gps = GPS()

		if args: self.path = args[0]
		else: self.path = self.fire_tour()

	def fire_tour(self):
		""""
		Generates a nearest neighbors tour through all fires on map.
		"""
		pos = self.start
		path = []
		for _ in range(len(self.gps.fires)):
			closest_fire = self.gps.nearest_fire(pos)
			self.gps.fires.remove(closest_fire)
			pos = closest_fire.coords
			path.append(closest_fire)
		return path
	
	def with_water(self):
		tour = Tour(self.path[:])
		i = tank = 0
		pos = tour.start
		while(i < len(tour.path)):
			#if tank <= 2:
			if tank < tour.path[i].time:
				water_pt = tour.gps.nearest_water_pt( LineString([pos, tour.path[i].coords]).centroid )
				target = min(60, sum(map(lambda x: x.time, tour.path[i:])))
				time_over_water = target-tank + 5
				tank = target
				tour.path.insert(i, Water(water_pt, time_over_water, target))
				pos = water_pt
				i += 1
			tank -= tour.path[i].time
			pos = tour.path[i].coords
			i += 1
		return tour

	def assess(self):
		i = total_area = 0
		run_time = 0
		while(i < len(self.path) and run_time < 440):
			run_time += (self.gps.travel_time(self.start, self.path[i].coords) if not i else self.gps.travel_time(self.path[i-1].coords, self.path[i].coords))
			run_time += self.path[i].time
			if (type(self.path[i]) == Fire):
				total_area += self.path[i].value()
			i += 1
		# print('RUN TIME', run_time)
		return total_area

def swap(fire_tour):
	tour = fire_tour[:]
	for i in range(4):
		i1 = random.randint(0, len(fire_tour)-1)
		i2 = random.randint(0, len(fire_tour)-1)
		tour[i1], tour[i2] = tour[i2], tour[i1]
	return Tour(tour)


class my_flight_controller(student_base):
	def __init__(self, tour):
		super().__init__()
		self.tour = tour
		self.gps = GPS()

	def message(self, text, i, pt):
		if type(pt) == Water: print(f'\n({i}) WATER:', text)
		else: print(f'\n({i}) FIRE:', text)

	def student_run(self, telemetry, commands):
		expected_time = 0

		#~~~~~~~~~SETUP & MOTION~~~~~~~~~
		def setup():
			print("Arming")
			self.arm()
			print("Taking off\n")
			self.start = (telemetry['latitude'], telemetry['longitude'])
			self.takeoff()
			time.sleep(3)

		def go_to(pt, i):
			lat, lon, = pt.coords.y, pt.coords.x
			self.goto(lat, lon, 10)
			time.sleep(1)
			tol = 0.0001
			err = np.linalg.norm([lat - telemetry['latitude'], lon - telemetry['longitude']])
			while err > tol:
				time.sleep(2)
				err = np.linalg.norm([lat - telemetry['latitude'], lon - telemetry['longitude']])
			self.message(f'arrived at ({round(lon,4)}, {round(lat,4)})', i, pt)
		
		def wait_over_fire(pt, next_fire, history):
			while	(
					history[-1] >= pt.target*1.1 and 
					history[-1] - history[-3] >= 1e-10 and 
					telemetry['water_pct_remaining'] >= 3
					):
				time.sleep(2)
				curr_area = round(telemetry["fire_polygons"][next_fire].area*1e10, 2)
				history.append(curr_area)		
				print('\nCurrent area:', curr_area)
				print('Target:', pt.target * 1e10)
			final_area = round(telemetry["fire_polygons"][next_fire].area*1e10, 2)
			print(f'\n~~~Suppressed fire, new area: {final_area}~~~\n\n')
			time.sleep(4)

		def wait_over_water(pt):
			target = (pt.target/60)*100 - 5
			while telemetry['water_pct_remaining'] <= target: time.sleep(2)
			print('\n~~~Replenished water~~~\n\n')

		def time_update(pt):
			nonlocal expected_time
			transit_time = self.gps.travel_time(Point([telemetry['longitude'], telemetry['latitude']]), pt.coords)
			wait_time = pt.time
			expected_time += transit_time + wait_time
			print('TRANSIT:', round(transit_time), '/  WAIT:', round(pt.time), '/  END:', round(expected_time))
			return transit_time + wait_time

		def execute_tour():
			nonlocal expected_time
			time.sleep(2)
			
			i = 0
			command_count = 1
			while(i < len(self.tour)):
				print(f'================= COMMAND #{command_count} =================')
				pt = self.tour[i]
				command_time = time_update(pt)
				lat, lon = round(pt.coords.y,4), round(pt.coords.x,4)

				if type(pt) == Fire:
					next_fire = next((i for i,f in enumerate(telemetry['fire_polygons']) if f.contains(pt.coords)), None)
					if not next_fire:
						print('~~~Skipped~~~\n\n')
						expected_time -= command_time
						command_count+=1
						i+=1
						continue
					start_area = telemetry["fire_polygons"][next_fire].area
					history = 3*[start_area]
					self.message(f'enroute to #{next_fire} / AREA = {round(start_area*1e10,2)} / ({lon}, {lat})', i, pt)
					go_to(pt, i)
					wait_over_fire(pt, next_fire, history)
				elif type(pt) == Water:
					self.message(f'enroute to ({lon}, {lat})', i, pt)
					go_to(pt, i)
					wait_over_water(pt)
				i+=1
				command_count += 1

		#~~~~~~~~COMMAND SEQUENCE~~~~~~~~
		setup()
		execute_tour()
		time.sleep(40)
		
		print("Landing"), self.land()
		while telemetry['in_air']:
			time.sleep(0.1)		
		print("Landed")

if __name__ == "__main__":
	temp = 3000
	best = Tour()
	best_eval = best.with_water().assess()
	print("BEST_EVAL", best_eval)
	curr, curr_eval = best, best_eval
	for i in range(1000):
		candidate = swap(best.path)
		cand_eval = candidate.with_water().assess()
		if cand_eval > best_eval:
			best, best_eval = candidate, cand_eval
			print(i, best_eval)
		diff = (cand_eval - curr_eval)/70
		te = temp / float(i+1)
		metropolis = math.exp(-diff / te)
		if diff > 0 or random.uniform(0,1) > metropolis:
			curr, curr_eval = candidate, cand_eval
	print('NEW BEST EVAL', best_eval)
	my_flight_controller(best.with_water().path).run()


	#input as longitude (x), latitude (y)
	# t = Tour()
	# fire_path = t.path
	# fire_water_path = t.with_water()
	# my_flight_controller(fire_water_path.path).run()

	# t = Tour()
	# total = 0
	# fire_path = t.path
	# for f in fire_path:
	# 	total += f.poly.area*1e10
	# 	print('AREA', f.poly.area*1e10, '\tTIME', f.time, '\tVALUE FOR TIME', f.value())
	# print(
	# 	t.with_water().assess()
	# )
	# print("TOTAL", total)


	

