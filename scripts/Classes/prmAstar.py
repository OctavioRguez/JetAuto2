#!/usr/bin/python3
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

class PRMAstar:
    def __init__(self) -> None:
        # Map parameters
        self.__samples = 350 # Number of points for PRM
        self.__maxDistance = 2.4 # Max distance between two points
        self.__num = 1000 # Points density for line collision check
        self.__x_limits = [0, 10] # Map x limits
        self.__y_limits = [0, 10] # Map y limits

        # Start and goal points
        self.__startPoint = (2, 8)
        self.__goalPoint = (5, 5)

        # Create obstacle map
        self.__obstacle_map = np.zeros((11, 11))
        self.__obstacle_map[3:8, 4] = 1  # Vertical obstacle
        self.__obstacle_map[8, 5:8] = 1  # Horizontal obstacle

    # Private function to calculate distance between two points
    def __dist(self, p1:list, p2:list) -> float:
        return np.linalg.norm(np.array(p2) - np.array(p1))

    # Check if point is inside an obstacle
    def __insideObstacle(self, point:tuple) -> bool:
        x, y = point
        return self.__obstacle_map[int(y), int(x)] == 1

    # Generate random points avoiding obstacles in map
    def __generatePoints(self) -> list:
        limits = [self.__x_limits, self.__y_limits]
        points = []
        while len(points) < self.__samples:
            sample = (np.random.randint(limits[0][0], limits[0][1]), 
                         np.random.randint(limits[1][0], limits[1][1]))
            if not self.__insideObstacle(sample):
                points.append(sample)
        return points

    # Build PRM graph considering map and avoiding obstacles
    def __PRM(self, points:list) -> nx.Graph:
        graph = nx.Graph()
        for point in points:
            graph.add_node(point)
            for neighbor in points:
                if point != neighbor and not self.__insideObstacle(point) and not self.__insideObstacle(neighbor):
                    dist = self.__dist(point, neighbor)
                    if dist < self.__maxDistance:
                        # Check if line between points crosses an obstacle
                        interpolation = np.linspace(point, neighbor, self.__num)
                        if all([not self.__insideObstacle(tuple(map(int, linePoint))) for linePoint in interpolation]):
                            graph.add_edge(point, neighbor, weight = dist)
        return graph

    # Public function to calculate PRM and A* path
    def calculate(self) -> None:
        # Generate random samples avoiding obstacles in map
        points = self.__generatePoints()
        graph = self.__PRM(points)

        # Find the closest points to start and goal
        start = min(points, key = lambda x: self.__dist(x, self.__startPoint))
        goal = min(points, key = lambda x: self.__dist(x, self.__goalPoint))

        # Find the path using A*
        path = nx.astar_path(graph, start, goal, heuristic = self.__dist)
        self.plot(graph, path, start, goal)

    # Public function to plot PRM and A* path
    def plot(self, graph:nx.Graph, path:list, start:list, goal:list) -> None:
        # Main figure
        plt.figure(figsize=(8, 8))

        # Plot map
        plt.imshow(self.__obstacle_map, cmap='Greys', origin='lower', 
                   extent=[self.__x_limits[0], self.__x_limits[1], self.__y_limits[0], self.__y_limits[1]])
        plt.title('Map with PRM and A* path')
        plt.xlabel('X Axis')
        plt.ylabel('Y Axis')

        # Plot PRM graph
        for edge in graph.edges():
            plt.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], color='blue', alpha=0.5)

        # Plot A* path
        x, y = zip(*path)
        plt.plot(x, y, color='red', linewidth=2, label='A* Path')
        plt.scatter(start[0], start[1], color = 'green', s = 100, label = 'Start')
        plt.scatter(goal[0], goal[1], color = 'purple', s = 100, label = 'Goal')
        plt.legend()
        plt.show()
