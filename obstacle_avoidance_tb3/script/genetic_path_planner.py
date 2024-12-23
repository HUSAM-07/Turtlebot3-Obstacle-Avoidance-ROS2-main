#!/usr/bin/env python3

import numpy as np
import random
from typing import List, Tuple
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

class GeneticPathPlanner(Node):
    def __init__(self):
        super().__init__('genetic_path_planner')
        
        # GA Parameters
        self.population_size = 100
        self.num_generations = 50
        self.mutation_rate = 0.1
        self.elite_size = 10
        
        # Map parameters
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.resolution = 0.0
        self.origin = None
        
        # ROS2 Publishers/Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
        self.path_pub = self.create_publisher(
            MarkerArray,
            'optimized_path',
            10)
            
        self.get_logger().info('Genetic Path Planner Node Initialized')

    def map_callback(self, msg: OccupancyGrid):
        """Store map data when received"""
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin
        
    def create_initial_population(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[List[Tuple[int, int]]]:
        """Generate initial population of paths"""
        population = []
        for _ in range(self.population_size):
            path = self.generate_random_path(start, goal)
            population.append(path)
        return population
    
    def generate_random_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Generate a single random path from start to goal"""
        path = [start]
        current = start
        while current != goal:
            next_point = self.get_random_next_point(current, goal)
            path.append(next_point)
            current = next_point
        return path
    
    def get_random_next_point(self, current: Tuple[int, int], goal: Tuple[int, int]) -> Tuple[int, int]:
        """Get random next point biased towards goal"""
        x, y = current
        gx, gy = goal
        
        # Bias movement towards goal
        dx = np.clip(gx - x, -1, 1)
        dy = np.clip(gy - y, -1, 1)
        
        if random.random() < 0.7:  # 70% chance to move towards goal
            new_x = x + dx
            new_y = y + dy
        else:  # 30% chance for random movement
            new_x = x + random.randint(-1, 1)
            new_y = y + random.randint(-1, 1)
            
        # Ensure within bounds
        new_x = np.clip(new_x, 0, self.map_width - 1)
        new_y = np.clip(new_y, 0, self.map_height - 1)
        
        return (int(new_x), int(new_y))
    
    def fitness(self, path: List[Tuple[int, int]]) -> float:
        """Calculate fitness of a path"""
        if not self.map_data is not None:
            return 0.0
            
        fitness_score = 0.0
        path_length = len(path)
        
        # Penalties
        obstacle_penalty = 100
        length_penalty = 0.1
        smoothness_penalty = 0.05
        
        # Check for collisions
        for point in path:
            x, y = point
            if self.map_data[y, x] > 50:  # If obstacle
                fitness_score -= obstacle_penalty
                
        # Penalize length
        fitness_score -= path_length * length_penalty
        
        # Penalize sharp turns
        for i in range(1, path_length - 1):
            prev = path[i-1]
            curr = path[i]
            next_point = path[i+1]
            
            # Calculate angle
            angle = self.calculate_angle(prev, curr, next_point)
            if angle < 90:  # Sharp turn
                fitness_score -= smoothness_penalty * (90 - angle)
                
        return fitness_score
    
    def calculate_angle(self, p1: Tuple[int, int], p2: Tuple[int, int], p3: Tuple[int, int]) -> float:
        """Calculate angle between three points"""
        import math
        
        a = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
        b = math.sqrt((p2[0]-p3[0])**2 + (p2[1]-p3[1])**2)
        c = math.sqrt((p3[0]-p1[0])**2 + (p3[1]-p1[1])**2)
        
        try:
            angle = math.degrees(math.acos((a**2 + b**2 - c**2)/(2*a*b)))
        except:
            angle = 0
            
        return angle
    
    def crossover(self, parent1: List[Tuple[int, int]], parent2: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """Perform crossover between two parents"""
        if len(parent1) < 2 or len(parent2) < 2:
            return parent1
            
        # Single point crossover
        point = random.randint(1, min(len(parent1), len(parent2))-1)
        child = parent1[:point] + parent2[point:]
        
        return child
    
    def mutate(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """Mutate a path"""
        if random.random() < self.mutation_rate:
            if len(path) < 3:
                return path
                
            # Select random point (not start/end)
            idx = random.randint(1, len(path)-2)
            
            # Mutate point slightly
            x, y = path[idx]
            new_x = x + random.randint(-1, 1)
            new_y = y + random.randint(-1, 1)
            
            # Ensure within bounds
            new_x = np.clip(new_x, 0, self.map_width - 1)
            new_y = np.clip(new_y, 0, self.map_height - 1)
            
            path[idx] = (int(new_x), int(new_y))
            
        return path
    
    def optimize_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Main optimization function using genetic algorithm"""
        if self.map_data is None:
            self.get_logger().warn('No map data available')
            return []
            
        # Initialize population
        population = self.create_initial_population(start, goal)
        
        for generation in range(self.num_generations):
            # Evaluate fitness
            fitness_scores = [(path, self.fitness(path)) for path in population]
            fitness_scores.sort(key=lambda x: x[1], reverse=True)
            
            # Select elite
            new_population = [path for path, _ in fitness_scores[:self.elite_size]]
            
            # Create rest of new population
            while len(new_population) < self.population_size:
                # Tournament selection
                parent1 = random.choice(fitness_scores[:50])[0]
                parent2 = random.choice(fitness_scores[:50])[0]
                
                # Crossover
                child = self.crossover(parent1, parent2)
                
                # Mutation
                child = self.mutate(child)
                
                new_population.append(child)
                
            population = new_population
            
            if generation % 10 == 0:
                self.get_logger().info(f'Generation {generation}, Best Fitness: {fitness_scores[0][1]}')
                
        # Return best path
        return fitness_scores[0][0]
    
    def visualize_path(self, path: List[Tuple[int, int]]):
        """Publish path as MarkerArray for visualization"""
        marker_array = MarkerArray()
        
        # Path line
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05  # Line width
        line_marker.color.a = 1.0
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        
        # Convert path points to world coordinates
        for point in path:
            x = point[0] * self.resolution + self.origin.position.x
            y = point[1] * self.resolution + self.origin.position.y
            
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            
            line_marker.points.append(p)
            
        marker_array.markers.append(line_marker)
        self.path_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = GeneticPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 