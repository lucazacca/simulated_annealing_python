from shapely.geometry import Point, Polygon
from operator import itemgetter
import matplotlib.pyplot as plt


class NodesInPoly:
    def __init__(self, polygon, obstacles, shift_x, shift_y, scan_dist):
        self.polygon = polygon
        self.obstacles = obstacles
        self.shift_x = shift_x
        self.shift_y = shift_y
        self.scan_dist = scan_dist
        self.node_distance = 2*scan_dist
        self.node_interval_offset = scan_dist/2.0
        self.x_max = max(polygon, key=itemgetter(0))[0] + self.node_distance
        self.y_max = max(polygon, key=itemgetter(1))[1] + self.node_distance
        self.x_min = min(polygon, key=itemgetter(0))[0] - self.node_distance
        self.y_min = min(polygon, key=itemgetter(1))[1] - self.node_distance
        self.x_range_meters = abs(self.x_max-self.x_min)
        self.y_range_meters = abs(self.y_max-self.y_min)
        self.x_nodes = int(self.x_range_meters/self.node_distance)
        self.y_nodes = int(self.y_range_meters/self.node_distance)
        
        print(f"X nodes: {self.x_nodes}, Y nodes: {self.y_nodes}")
        self.polygon_shapely = Polygon(self.polygon)
        self.optimization_index = None

        self.better_coverage()
    
    def better_coverage(self):
        self.mega_nodes_count = 0
        self.mega_nodes = [[[0, 0, 0] for _ in range(self.y_nodes)] for _ in range(self.x_nodes)]
        

        for i in range(self.x_nodes):
            for j in range(self.y_nodes):
                self.mega_nodes[i][j][0] = self.x_min + i * self.node_distance + self.shift_x
                self.mega_nodes[i][j][1] = self.y_min + j * self.node_distance + self.shift_y
                
                # mega_node = Point(self.mega_nodes[i][j][0], self.mega_nodes[i][j][1])
                # if self.polygon_shapely.contains(mega_node):
                #     self.mega_nodes[i][j][2] = 0
                #     self.mega_nodes_count += 1
                # else:
                #     self.mega_nodes[i][j][2] = 1
        
        
        
        # print("Number of sub-nodes that will be used for trajectories:", 4.0 * self.mega_nodes_count)
        
        self.sub_nodes = [[[0, 0, 0] for _ in range(2 * self.y_nodes)] for _ in range(2 * self.x_nodes)]
        
        for i in range(self.x_nodes):
            for j in range(self.y_nodes):
                self.sub_nodes[2 * i][2 * j + 1][0] = self.mega_nodes[i][j][0] - self.node_interval_offset
                self.sub_nodes[2 * i][2 * j + 1][1] = self.mega_nodes[i][j][1] + self.node_interval_offset

                self.sub_nodes[2 * i + 1][2 * j + 1][0] = self.mega_nodes[i][j][0] + self.node_interval_offset
                self.sub_nodes[2 * i + 1][2 * j + 1][1] = self.mega_nodes[i][j][1] + self.node_interval_offset

                self.sub_nodes[2 * i][2 * j][0] = self.mega_nodes[i][j][0] - self.node_interval_offset
                self.sub_nodes[2 * i][2 * j][1] = self.mega_nodes[i][j][1] - self.node_interval_offset

                self.sub_nodes[2 * i + 1][2 * j][0] = self.mega_nodes[i][j][0] + self.node_interval_offset
                self.sub_nodes[2 * i + 1][2 * j][1] = self.mega_nodes[i][j][1] - self.node_interval_offset

                self.check_in_poly_and_obstacle(i, j)

                self.sub_nodes[2 * i][2 * j + 1][2] = self.mega_nodes[i][j][2]
                self.sub_nodes[2 * i + 1][2 * j + 1][2] = self.mega_nodes[i][j][2]
                self.sub_nodes[2 * i][2 * j][2] = self.mega_nodes[i][j][2]
                self.sub_nodes[2 * i + 1][2 * j][2] = self.mega_nodes[i][j][2]
        
        print("Number of mega-nodes inside polygon:", self.mega_nodes_count)
        print("\n")

    def check_in_poly_and_obstacle(self, i, j):
        sub_node_1 = Point(self.sub_nodes[2 * i][2 * j + 1][0], self.sub_nodes[2 * i][2 * j + 1][1])
        sub_node_2 = Point(self.sub_nodes[2 * i + 1][2 * j + 1][0], self.sub_nodes[2 * i + 1][2 * j + 1][1])
        sub_node_3 = Point(self.sub_nodes[2 * i][2 * j][0], self.sub_nodes[2 * i][2 * j][1])
        sub_node_4 = Point(self.sub_nodes[2 * i + 1][2 * j][0], self.sub_nodes[2 * i + 1][2 * j][1])
        mega_node = Point(self.mega_nodes[i][j][0], self.mega_nodes[i][j][1])

        if (self.polygon_shapely.contains(sub_node_1) or self.polygon_shapely.contains(sub_node_2) or 
            self.polygon_shapely.contains(sub_node_3) or self.polygon_shapely.contains(sub_node_4)):

            self.mega_nodes[i][j][2] = 0
            self.mega_nodes_count += 1
        else:
            self.mega_nodes[i][j][2] = 1

        if len(self.obstacles) > 0:
            for k in range(len(self.obstacles)):
                polygon_obstacle_shapely = Polygon(self.obstacles[k])

                if (polygon_obstacle_shapely.contains(sub_node_1) or polygon_obstacle_shapely.contains(sub_node_2) or
                     polygon_obstacle_shapely.contains(sub_node_3) or polygon_obstacle_shapely.contains(sub_node_4) or
                     polygon_obstacle_shapely.contains(mega_node)):    
                    
                    self.mega_nodes[i][j][2] = 1
                    self.mega_nodes_count -= 1
    
    def visualization(self):
        x_meganodes_free = []
        y_meganodes_free = []
        x_nodes_free = []
        y_nodes_free = []
        x_meganodes_occupied = []
        y_meganodes_occupied = []
        x_nodes_occupied = []
        y_nodes_occupied = []

        for j in range(self.y_nodes):
            for i in range(self.x_nodes):
                if self.mega_nodes[i][j][2] == 0:
                    x_meganodes_free.append(self.mega_nodes[i][j][0])
                    y_meganodes_free.append(self.mega_nodes[i][j][1])
                else:
                    x_meganodes_occupied.append(self.mega_nodes[i][j][0])
                    y_meganodes_occupied.append(self.mega_nodes[i][j][1])
                
                x = self.sub_nodes[2 * i][2 * j + 1][0]
                y = self.sub_nodes[2 * i][2 * j + 1][1]

                if self.sub_nodes[2 * i][2 * j + 1][2] == 0:
                    x_nodes_free.append(x)
                    y_nodes_free.append(y)
                else:
                    x_nodes_occupied.append(x)
                    y_nodes_occupied.append(y)

                x = self.sub_nodes[2 * i + 1][2 * j + 1][0]
                y = self.sub_nodes[2 * i + 1][2 * j + 1][1]

                if self.sub_nodes[2 * i + 1][2 * j + 1][2] == 0:
                    x_nodes_free.append(x)
                    y_nodes_free.append(y)
                else:
                    x_nodes_occupied.append(x)
                    y_nodes_occupied.append(y)
                
                x = self.sub_nodes[2 * i][2 * j][0]
                y = self.sub_nodes[2 * i][2 * j][1]

                if self.sub_nodes[2 * i][2 * j][2] == 0:
                    x_nodes_free.append(x)
                    y_nodes_free.append(y)
                else:
                    x_nodes_occupied.append(x)
                    y_nodes_occupied.append(y)
                
                x = self.sub_nodes[2 * i + 1][2 * j][0]
                y = self.sub_nodes[2 * i + 1][2 * j][1]

                if self.sub_nodes[2 * i + 1][2 * j][2] == 0:
                    x_nodes_free.append(x)
                    y_nodes_free.append(y)
                else:
                    x_nodes_occupied.append(x)
                    y_nodes_occupied.append(y)
                
        
        fig, ax = plt.subplots()
        for k in range(len(self.obstacles)):
            obstacle_shapely = Polygon(self.obstacles[k])
            x_obstacle, y_obstacle = obstacle_shapely.exterior.xy
            ax.plot(x_obstacle, y_obstacle, color = "black")

        x_polygon, y_polygon = self.polygon_shapely.exterior.xy
        ax.plot(x_polygon, y_polygon, color="blue")
        
        ax.scatter(x_meganodes_free, y_meganodes_free, color="green")
        ax.scatter(x_meganodes_occupied, y_meganodes_occupied, color="red")
        ax.scatter(x_nodes_free, y_nodes_free, color="green", marker="+", linewidths=0.2)
        ax.scatter(x_nodes_occupied, y_nodes_occupied, color="red", marker="+", linewidths=0.2)

        plt.show()


    def get_optimization_index(self):
        if self.optimization_index is None:
            a = 0.68
            b = 0.32
            c = 0.25

            polygon_area = self.get_polygon_area_without_obstacles()
            nodes_in_term = (self.mega_nodes_count * (self.node_distance ** 2)) / polygon_area
            min_bb_area_term = polygon_area / (self.x_range_meters*self.y_range_meters)
            
            if self.mega_nodes_count > 0:
                equal_margins_term = self.margin_norm_ssi()
            else:
                equal_margins_term = 1

            self.optimization_index = a * nodes_in_term + b * min_bb_area_term - c * equal_margins_term

        return self.optimization_index

    def get_polygon_area(self):
        sum_value = 0
        for i in range(len(self.polygon)):
            if i == 0:
                sum_value += self.polygon[i][0] * (self.polygon[i + 1][1] - self.polygon[len(self.polygon) - 1][1])
            elif i == len(self.polygon) - 1:
                sum_value += self.polygon[i][0] * (self.polygon[0][1] - self.polygon[i - 1][1])
            else:
                sum_value += self.polygon[i][0] * (self.polygon[i + 1][1] - self.polygon[i - 1][1])
        
        area = 0.5 * abs(sum_value)
        return area

    def get_polygon_area_without_obstacles(self):
        polygon_area = self.get_polygon_area()
        
        obstacles_area = 0
        if len(self.obstacles) > 0:
            for k in range(len(self.obstacles)):
                sum_value = 0
                for i in range(len(self.obstacles[k])):
                    if i == 0:
                        sum_value += self.obstacles[k][i][0] * (self.obstacles[k][i + 1][1] - self.obstacles[k][len(self.obstacles[k]) - 1][1])
                    elif i == len(self.obstacles[k]) - 1:
                        sum_value += self.obstacles[k][i][0] * (self.obstacles[k][0][1] - self.obstacles[k][i - 1][1])
                    else:
                        sum_value += self.obstacles[k][i][0] * (self.obstacles[k][i + 1][1] - self.obstacles[k][i - 1][1])
                
                obstacles_area += 0.5 * abs(sum_value)
        
        return polygon_area - obstacles_area
    

    def margin_norm_ssi(self):
        coords = []
        for i in range(2 * self.x_nodes):
            for j in range(2 * self.y_nodes):
                if self.sub_nodes[i][j][2] != 1:
                    coords.append([self.sub_nodes[i][j][0], self.sub_nodes[i][j][1]])

        x_box_max = max(coords, key=itemgetter(0))[0]
        x_box_min = min(coords, key=itemgetter(0))[0]
        y_box_max = max(coords, key=itemgetter(1))[1]
        y_box_min = min(coords, key=itemgetter(1))[1]

        SSI = (abs(abs(x_box_max - self.x_max) - abs(self.x_min - x_box_min)) / (2 * abs(x_box_max - x_box_min))) + \
            (abs(abs(y_box_max - self.y_max) - abs(self.y_min - y_box_min)) / (2 * abs(y_box_max - y_box_min)))

        return SSI



def tests():
    # # Test case 1
    # polygon = [(0, 0), (5, 0), (5, 5), (0, 5)]  # Square polygon
    # obstacles = [[(1, 1), (2, 1), (2, 2), (1, 2)]]  # Single obstacle inside the square
    # shift_x = 0
    # shift_y = 0
    # scan_dist = 1

    # self = NodesInPoly(polygon, obstacles, shift_x, shift_y, scan_dist)
    # self.better_coverage()
    # print(self.mega_nodes)
    # print(self.sub_nodes)

    # # Test case 2
    # polygon = [(0, 0), (10, 0), (10, 5), (5, 5), (5, 10), (0, 10)]  # Irregular polygon
    # obstacles = [[(1, 1), (2, 1), (2, 2), (1, 2)], [(7, 3), (8, 3), (8, 4), (7, 4)]]  # Two obstacles inside the polygon
    # shift_x = 2
    # shift_y = 2
    # scan_dist = 2

    # self = NodesInPoly(polygon, obstacles, shift_x, shift_y, scan_dist)
    # self.better_coverage()

    # # Test case 3
    # polygon = [(0, 0), (10, 0), (10, 10), (0, 10)]  # Square polygon
    # obstacles = []  # No obstacles
    # shift_x = -2
    # shift_y = -2
    # scan_dist = 2

    # self = NodesInPoly(polygon, obstacles, shift_x, shift_y, scan_dist)
    # self.better_coverage()
    # gpd.GeoSeries([self.polygon_shapely]).boundary.plot()
    # plt.show()


    # polygon = [(0, 0), (20, 0), (20, 30), (10, 20), (0, 30)]  # Irregular polygon
    # obstacles = [[(8, 8), (12, 8), (12, 12), (8, 12)]]  # Single obstacle inside the polygon

    # polygon = [(2,1), (11, 2), (13, 6), (9, 10), (7,8), (5, 9), (5, 5)]
    # obstacles = [[(7, 4), (8, 6), (6, 6)], [(9, 3), (11, 4), (10, 5)]]
    shift_x = 0.5
    shift_y = 0.5
    scan_dist = 0.5
    polygon = [[2,1], [11, 2], [13, 6], [9, 10], [7,8], [5, 9], [5, 5]]
    obstacles = [[[7, 4], [8, 6], [6, 6]], [[9, 3], [11, 4], [10, 5]]]
    scan_dist = 0.5
    nodes_in_poly = NodesInPoly(polygon, obstacles, shift_x, shift_y, scan_dist)
    nodes_in_poly.better_coverage()
    nodes_in_poly.visualization()




def main():
    # p1 = Point(2.5, 2)
    
    # coords = [(1, 1), (2, 3), (4, 2), (3, 5), (1.5, 4)]
    # poly = Polygon(coords)

    # print(poly.contains(p1))   # True
    tests()

if __name__ == '__main__':
    main()
