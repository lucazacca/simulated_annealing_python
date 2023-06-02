import math
import random
from nodes_in_poly import NodesInPoly
from operator import itemgetter

class Transformations:
    def __init__(self, theta, shift_x, shift_y, visualization):
        self.shift_x = shift_x
        self.shift_y = shift_y
        self.theta = theta
        self.rotated_polygon = None
        self.rotated_obstacles = None
        self.visualization = visualization

    def rotate_polygon(self, polygon):
        rotated_polygon = []
        x_max = max(polygon, key=itemgetter(0))[0]
        y_max = max(polygon, key=itemgetter(1))[1]
        x_min = min(polygon, key=itemgetter(0))[0]
        y_min = min(polygon, key=itemgetter(1))[1]
        center_x = (x_max+x_min)/2
        center_y = (y_max+y_min)/2
        for i in range(len(polygon)):
            x, y = polygon[i]
            x = x-center_x
            y = y-center_y
            x_rot = x * math.cos(math.radians(self.theta)) - y * math.sin(math.radians(self.theta)) + center_x
            y_rot = x * math.sin(math.radians(self.theta)) + y * math.cos(math.radians(self.theta)) + center_y
            rotated_polygon.append([x_rot, y_rot])

        return rotated_polygon

    def rotate_and_shift(self, polygon, obstacles, scan_dist):
        self.rotated_polygon = self.rotate_polygon(polygon)
        
        if len(obstacles)>0:
            self.rotated_obstacles = []
            for i in range(len(obstacles)):
                self.rotated_obstacles.append(self.rotate_polygon(obstacles[i]))

        nodes_in_poly = NodesInPoly(polygon=self.rotated_polygon, obstacles=self.rotated_obstacles, shift_x=self.shift_x, shift_y=self.shift_y, scan_dist=scan_dist)
        
        if self.visualization:
            nodes_in_poly.visualization()

        optimization_index = nodes_in_poly.get_optimization_index()
        return optimization_index


        

class Solution:
        def __init__(self, polygon, obstacles, scan_dist):
            self.theta = 0
            self.shift_x = scan_dist
            self.shift_y = scan_dist
            self.polygon = polygon
            self.obstacles = obstacles
            self.rotated_polygon = None
            self.rotated_obstacles = None
            self.scan_dist = scan_dist
            self.optimization_index = 0

        def random(self):
            self.theta = random.random() * 90
            self.shift_x = random.random() * 2 * self.scan_dist
            self.shift_y = random.random() * 2 * self.scan_dist

        def create_new(self):
            random_sol = Transformations(self.theta, self.shift_x, self.shift_y, visualization=False)
            self.optimization_index = random_sol.rotate_and_shift(self.polygon, self.obstacles, self.scan_dist)
            self.rotated_polygon = random_sol.rotated_polygon
            self.rotated_obstacles = random_sol.rotated_obstacles

        def visualize_solution(self):
            current = Transformations(self.theta, self.shift_x, self.shift_y, visualization=True)
            current.rotate_and_shift(self.polygon, self.obstacles, self.scan_dist)

            




class SimulatedAnnealing:
    def __init__(self, polygon, obstacles, scan_dist):
        self.polygon = polygon
        self.obstacles = obstacles
        self.scan_dist = scan_dist
        self.optimal_theta = 0
        self.optimal_shift_x = 0
        self.optimal_shift_y = 0
        self.optimization_index_max = 0
        self.optimization_index_current = 0

    def run(self):
        T = 1000  # Initial temperature
        Tmin = 5  # Minimum temperature
        alpha = 0.4  # Cooling rate
        num_iterations = 20  # Number of iterations

        current_sol = Solution(self.polygon, self.obstacles, self.scan_dist)
        
        current_sol.create_new()

        while (T > Tmin):
            for _ in range(num_iterations):
                self.optimization_index_current = current_sol.optimization_index

                if self.optimization_index_current > self.optimization_index_max:
                    self.optimization_index_max = self.optimization_index_current
                    self.optimal_theta = current_sol.theta
                    self.optimal_shift_x = current_sol.shift_x
                    self.optimal_shift_y = current_sol.shift_y

                new_sol = Solution(polygon=self.polygon, obstacles=self.obstacles, scan_dist=self.scan_dist)
                new_sol.random()
                new_sol.create_new()

                ap = math.exp((self.optimization_index_current - new_sol.optimization_index) / T)
                if ap > random.random():
                    current_sol = new_sol
                    # current_sol.visualize_solution()

            T *= alpha

        print("Final value of optimization index:", self.optimization_index_max)
