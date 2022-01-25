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
	"""
	Represents a location on a tour.
	"""
	def __init__(self, coords, time, target):
		self.coords = coords
		self.time = time
		self.target = target

class Fire(Waypoint):
	"""
	Represents fire on a tour with additional poly property (a Shapely polygon).
	"""
	def __init__(self, poly):
		self.poly = poly
		super().__init__(poly.centroid, self.time(), self.target_area())
	
	def time(self):
		"""
		Computes effective time spent at fire (used to calculate amount of water deployed).
		Accounts for suppression of fire before drone reaches centroid and delay in response to commands.
		"""
		transit = 5
		if self.poly.area >= self.target_area(): transit += math.log(self.target_area() / self.poly.area) / math.log(.95) / 5.189
		return transit
	
	def target_area(self):
		"""
		Intended area of suppressed fires (not scaled by 1 * 10^10)
		"""
		return 1e-8

	def value(self):
		"""
		Computes point value of spending time at fire,
		scaled by factor of 5.189 for accuracy (determined empirically).
		"""
		return round( (self.poly.area - self.poly.area * math.pow(.95, self.time*5.189))*1e10 )

class Water(Waypoint):
	"""
	Represents water on a tour.
	"""
	def __init__(self, coords, time, target):
		super().__init__(coords, time, target)

class GPS():
	"""
	Class for finding geometric relationships between drone and map objects.
	"""
	def __init__(self):
		self.home = Point(-70.6185, 42.98575)
		self.water = gpd.read_file('./data/waterbodies.geojson')['geometry']
		self.fires = self._load_fires()

	def _load_fires(self):
		"""
		Loads fires from map into list of Fire objects.
		"""
		f = open('maps/Fire_Competition.json')
		data = json.load(f)
		shapes = zip(data['data_fs']['xs'], data['data_fs']['ys'])
		return [Fire(Polygon([*zip(*shape)])) for shape in shapes]

	def travel_time(self, p1, p2):
		"""
		Computes time taken to travel from p1 to p2 using t = d / v.
		v = 1.1976737e-4, determined empirically.
		"""
		return p1.distance(p2) / 1.1976737e-4

	def nearest_fire(self, pos):
		"""
		Returns Fire closest to pos.
		"""
		return min(self.fires, key=lambda f: pos.distance(f.coords))
	
	def nearest_water_pt(self, pos):
		"""
		Returns water point closest to pos.
		"""
		points = self.water.apply(lambda x: nearest_points(x,pos)[0])
		return min(points, key=lambda x: pos.distance(x))
	
	def largest_fire(self, fires):
		"""
		Returns largest fire by area from the list found in telemetry.
		"""
		return max(fires, key=lambda f: f.area)
	
	def largest_fires(self, fires):
		f = fires[:]
		f.sorted(reverse=True, key=lambda f: f.area)
		return f
	
	def nearest_fire_pt(self,pt,fires):
		return min(self.fires, key=lambda f: pt.distance(f.centroid))

		
class Tour():
	"""
	Class for generating and storing tours across the map.
	"""
	def __init__(self, *args):
		self.start = Point(-70.6185, 42.98575) #starting location
		self.gps = GPS()

		if args: self.path = args[0]
		else: self.path = self.fire_tour()

	def fire_tour(self):
		""""
		Generates a nearest neighbors tour through all fires on map (disregards water).
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
		"""
		Adds water to nearest-neighbors fire tour by simulating a run offline.
		Starts with empty tank, fills to 60, reduces by appropriate amount at each fire,
		then adds a water visit to the tour when replenishing is necessary.
		"""
		tour = Tour(self.path[:])
		i = tank = 0
		pos = tour.start
		while(i < len(tour.path)):
			if tank < 8:
			#if tank < tour.path[i].time:
				water_pt = tour.gps.nearest_water_pt( LineString([pos, tour.path[i].coords]).centroid )
				target = min(60, sum(map(lambda x: x.time, tour.path[i:])))
				time_over_water = target - tank + 5
				tank = target
				tour.path.insert(i, Water(water_pt, time_over_water, target))
				pos = water_pt
				i += 1
			tank -= tour.path[i].time + 1.35
			pos = tour.path[i].coords
			i += 1
		return tour

	def assess(self):
		"""
		Computes the fitness of a tour (metric: area extinguished)
		Run time is capped at < 600 seconds to minimize the risk of exceeding 10 minutes and
		to prioritize larger fires.
		"""
		i = total_area = 0
		run_time = 0
		while(i < len(self.path) and run_time < 600):
			run_time += (self.gps.travel_time(self.start, self.path[i].coords) if not i else self.gps.travel_time(self.path[i-1].coords, self.path[i].coords))
			run_time += self.path[i].time
			if (type(self.path[i]) == Fire):
				total_area += self.path[i].value()
			i += 1
		# print('RUN TIME', run_time)
		return total_area

def swap(fire_tour):
	"""
	Returns a new Tour with the order of 4 fires swapped.
	"""
	tour = fire_tour[:]
	for _ in range(2):
		i1 = random.randint(0, len(fire_tour)-1)
		i2 = random.randint(0, len(fire_tour)-1)
		tour[i1], tour[i2] = tour[i2], tour[i1]
	return Tour(tour)


class my_flight_controller(student_base):
	"""
	Manages drone movement and makes fine adjustments to pre-planned route in real-time.
	"""
	def __init__(self, tour):
		super().__init__()
		self.tour = tour
		self.gps = GPS()

	def message(self, text, i, pt_type):
		"""
		Useful method for printing structured updates to terminal.
		"""
		print(f'\n({i}) {pt_type}:', text)

	def student_run(self, telemetry, commands):
		expected_time = 5

		#~~~~~~~~~SETUP & MOTION~~~~~~~~~
		def setup():
			"""
			Prepares drone for tour.
			"""
			print("Arming")
			time.sleep(.1)
			self.arm()
			print("Taking off\n")
			time.sleep(.1)
			self.start = (telemetry['latitude'], telemetry['longitude'])
			self.takeoff()
			time.sleep(.5)

		def go_to(coords, i, pt_type):
			"""
			Moves drone to specified coords with reporting of updates to terminal.
			"""
			lat, lon, = coords.y, coords.x
			time.sleep(.2)
			self.goto(lat, lon, 10)
			time.sleep(.2)
			tol = 0.0001
			err = np.linalg.norm([lat - telemetry['latitude'], lon - telemetry['longitude']])
			time.sleep(.2)
			while err > tol:
				time.sleep(2)
				err = np.linalg.norm([lat - telemetry['latitude'], lon - telemetry['longitude']])
				time.sleep(.2)
			self.message(f'arrived at ({round(lon,4)}, {round(lat,4)})', i, pt_type)
			time.sleep(.1)
		
		def wait_over_fire(pt, next_fire, history,i):
			"""
			Defines duration that drone spends over fire with 3 end conditions calculated live:
				(1) Target fire area is reached.
				(2) Fire area is no longer changing
				(3) Water is depleted.
			Also readjusts drone position to the fire's new centroid as its area is reduced.
			Accounts for lag in response to commands.
			"""
			while	(
					history[-1] >= pt.target*1.1 and 
					history[-1] - history[-3] >= 1e-10 and 
					telemetry['water_pct_remaining'] >= 3
					):
				time.sleep(2)
				curr_area = round(telemetry['fire_polygons'][next_fire].area*1e10, 2)
				history.append(curr_area)
				time.sleep(.2)
				print('\nCurrent area:', curr_area)
				print('Target:', pt.target * 1e10)
			try:
				go_to(telemetry['fire_polygons'][next_fire].centroid, i, 'NEW CENTROID')
				time.sleep(.1)
			except:
				print('Centroid not found')
			else:
				try:
					final_area = round(telemetry["fire_polygons"][next_fire].area*1e10, 2)
				except:
					print()
				else:
					time.sleep(.2)
					print(f'\n~~~Suppressed fire, new area: {final_area}~~~\n\n')
			time.sleep(.9)
			
		def wait_over_water(pt):
			"""
			Defines duration drone spends over water. 
			Accounts for lag in response to commands.
			"""
			time.sleep(.5)
			target = (pt.target - (telemetry['water_pct_remaining']*.6)) - 5
			time.sleep(.5)
			print(pt.target)
			print( (telemetry['water_pct_remaining']*.6) )
			time.sleep(50)
			time.sleep(.5)
			print('\n~~~Replenished water~~~\n\n')
			time.sleep(.6)

		def time_update(pt):
			"""
			Updates clock with values from offline predictions.
			"""
			nonlocal expected_time
			transit_time = self.gps.travel_time(Point([telemetry['longitude'], telemetry['latitude']]), pt.coords)
			wait_time = pt.time
			expected_time += transit_time + wait_time
			print('TRANSIT:', round(transit_time), '/  WAIT:', round(pt.time), '/  END:', round(expected_time))
			return transit_time + wait_time

		def execute_tour():
			"""
			Sends drone on tour created from Simulated Annealing with a Nearest Neighbors initial path.
			"""
			nonlocal expected_time
			predicted_tank = 0
			predicted_suppressed = 0
			time.sleep(2)
			
			i = 0
			command_count = 1
			while(i < len(self.tour)):
				print(f'================= COMMAND #{command_count} =================')
				pt = self.tour[i]
				command_time = time_update(pt)
				lat, lon = round(pt.coords.y,4), round(pt.coords.x,4)

				time.sleep(.2)

				if type(pt) == Fire:
					#determining index of fire in Visualizer for live calculations later
					next_fire = next((i for i,f in enumerate(telemetry['fire_polygons']) if f.envelope.contains(pt.coords)), None)
					
					#safety net if centroid calculation fails
					if not next_fire:
						print('~~~Skipped~~~\n\n')
						expected_time -= command_time
						command_count+=1
						i+=1
						continue
					
					start_area = telemetry["fire_polygons"][next_fire].area
					history = 3*[start_area]
					time.sleep(.2)

					#heading to next fire
					self.message(f'enroute to #{next_fire} / AREA = {round(start_area*1e10,2)} / ({lon}, {lat})', i, 'FIRE')
					go_to(pt.coords, i, 'FIRE')
					wait_over_fire(pt, next_fire, history, i)
					time.sleep(.2)

					predicted_tank -= pt.time
					predicted_suppressed += pt.value()
					time.sleep(.2)

					print('Predicted tank', predicted_tank)
					print('Predicted suppressed', round(((predicted_suppressed)/13003)*100))

				elif type(pt) == Water:
					time.sleep(.2)
					self.message(f'enroute to ({lon}, {lat})', i, 'WATER')
					go_to(pt.coords, i, 'WATER')
					time.sleep(.2)
					wait_over_water(pt)
					time.sleep(.2)
					predicted_tank = pt.target

					print('Predicted tank:', predicted_tank)
				i+=1
				command_count += 1
				time.sleep(3)

		#~~~~~~~~COMMAND SEQUENCE~~~~~~~~
		setup()
		execute_tour()

def anneal(initial, temp, iterations):
	"""
	Performs simulated annealing on initial path with given
	temperature and number of iterations.
	"""
	temp = 20
	best = initial
	best_eval = best.with_water().assess()
	print("BEST_EVAL", best_eval)
	curr, curr_eval = best, best_eval
	for i in range(iterations):
		candidate = swap(best.path)
		cand_eval = candidate.with_water().assess()
		if cand_eval > best_eval:
			best, best_eval = candidate, cand_eval
			print(i, best_eval)
		diff = (cand_eval - curr_eval)
		temp *= .995
		metropolis = math.exp(-abs(diff / temp))
		if diff > 0 or metropolis > random.random():
			curr, curr_eval = candidate, cand_eval
		if i % 100 == 0: print(i)
	print('NEW BEST EVAL', best_eval)
	return best.with_water().path

if __name__ == "__main__":
	nearest_neighbors = Tour()
	opt_path = anneal(nearest_neighbors, 20, 500)
	my_flight_controller(opt_path).run()

