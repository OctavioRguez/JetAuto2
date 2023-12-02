#!/usr/bin/python3
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

class PRMAstar:
    def __init__(self, samples:int, maxDist:float, density:int) -> None:
        # Map parameters
        self.__pointsPerIteration = samples # Number of points for PRM
        self.__maxDistance = maxDist # Max distance between two points
        self.__num = density # Points density for line collision check
        self.__xLimits = None # Map x limits
        self.__yLimits = None # Map y limits

        # Start and goal points
        self.__startPoint = None
        self.__goalPoint = (40, 40)

        # Obstacle map
        self.__map = None
        # PRM results
        self.__graph = nx.Graph()
        self.__points = []

    # Private function to calculate distance between two points
    def __dist(self, p1:list, p2:list) -> float:
        return np.linalg.norm(np.array(p2) - np.array(p1))

    # Check if point is inside an obstacle
    def __insideObstacle(self, point:tuple) -> bool:
        return any(self.__map[int(y), int(x)] == 100 for x, y in self.__neighborhood(point))
 
    # Create kernel of points around current point
    def __neighborhood(self, point:tuple) -> list:
        kernel = []
        for i in range(-10, 11):
            for j in range(-10, 11):
                x, y = point[0]+i, point[1]+j
                if (self.__xLimits[0] <= x <= self.__xLimits[1] and self.__yLimits[0] <= y <= self.__yLimits[1]):
                    kernel.append((x, y))
        return kernel

    # Generate random points avoiding obstacles in map
    def __generatePoints(self) -> list:
        points = []
        while len(points) < self.__pointsPerIteration:
            sample = (np.random.uniform(self.__xLimits[0], self.__xLimits[1]), 
                      np.random.uniform(self.__yLimits[0], self.__yLimits[1]))
            if not self.__insideObstacle(sample) and self.__map[int(sample[1]), int(sample[0])] != -1:
                points.append(sample)
                self.__points.append(sample)
        return points

    # Build PRM graph considering map and avoiding obstacles
    def __PRM(self, points:list) -> None:
        for point in points:
            self.__graph.add_node(point)
            for neighbor in points:
                if point != neighbor:
                    dist = self.__dist(point, neighbor)
                    if dist < self.__maxDistance:
                        # Check if line between points crosses an obstacle
                        interpolation = np.linspace(point, neighbor, self.__num)
                        if any(self.__insideObstacle(linePoint) == True for linePoint in interpolation):
                            continue
                        self.__graph.add_edge(point, neighbor, weight = dist)

    # Public function to calculate PRM and A* path
    def calculate(self, map:np.ndarray, limits:list) -> list:
        self.__map = map
        self.__xLimits = limits[0]
        self.__yLimits = limits[1]
    
        # Generate random samples avoiding obstacles in map
        points = self.__generatePoints()
        self.__PRM(points)

    def findPath(self, start:list) -> list:
        self.__startPoint = start

        # Find the closest points to start and goal
        start = min(self.__points, key = lambda x: self.__dist(x, self.__startPoint))
        goal = min(self.__points, key = lambda x: self.__dist(x, self.__goalPoint))

        # Find the path using A*
        path = nx.astar_path(self.__graph, start, goal, heuristic = self.__dist)
        self.__plot(path, start, goal)
        return path

    # Public function to plot PRM and A* path
    def __plot(self, path:list, start:list, goal:list) -> None:
        # Main figure
        plt.figure(figsize=(12, 12))

        # Plot map
        plt.imshow(self.__map, cmap='Greys', origin='lower')
        plt.title('Map with PRM and A* path')
        plt.xlabel('X Axis')
        plt.ylabel('Y Axis')

        # Plot PRM graph
        for edge in self.__graph.edges():
            plt.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], color='blue')

        # Plot A* path
        x, y = zip(*path)
        plt.plot(x, y, color='red', linewidth=2, label='A* Path')
        plt.scatter(start[0], start[1], color = 'green', s = 200, label = 'Start')
        plt.scatter(goal[0], goal[1], color = 'purple', s = 200, label = 'Goal')
        plt.legend()
        plt.savefig('/home/tavo/PRM.png')
        print("Saving PRM.png")
