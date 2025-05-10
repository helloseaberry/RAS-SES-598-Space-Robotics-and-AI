#!/usr/bin/env python3

import math
import random
import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
from turtlesim.msg import Pose

class NodeRRT:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')
        self.teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')

        self.start = NodeRRT(1.0, 1.0)
        self.goal = NodeRRT(10.0, 10.0)
        self.obstacles = [NodeRRT(5.5, 5.5), NodeRRT(3.5, 7.0), NodeRRT(6.0, 3.5)]
        self.radius = 1.0
        self.nodes = self.rrt(self.start, self.goal, 500)
        self.path = self.extract_path()
        self.draw_path()

    def distance(self, n1, n2):
        return math.hypot(n1.x - n2.x, n1.y - n2.y)

    def steer(self, from_node, to_node, extend_length=0.5):
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_x = from_node.x + extend_length * math.cos(theta)
        new_y = from_node.y + extend_length * math.sin(theta)
        new_node = NodeRRT(new_x, new_y)
        new_node.parent = from_node
        return new_node

    def collision(self, node):
        for obs in self.obstacles:
            if self.distance(node, obs) < self.radius:
                return True
        return False

    def rrt(self, start, goal, max_iter):
        nodes = [start]
        for _ in range(max_iter):
            rand = NodeRRT(random.uniform(0.5, 10.5), random.uniform(0.5, 10.5))
            nearest = min(nodes, key=lambda n: self.distance(n, rand))
            new_node = self.steer(nearest, rand)
            if not self.collision(new_node):
                nodes.append(new_node)
                if self.distance(new_node, goal) < 1.0:
                    goal.parent = new_node
                    nodes.append(goal)
                    break
        return nodes

    def extract_path(self):
        path = []
        node = self.goal
        while node.parent:
            path.append((node.x, node.y))
            node = node.parent
        path.reverse()
        return path

    def draw_path(self):
        self.get_logger().info("Drawing path...")
        for (x, y) in self.path:
            req = TeleportAbsolute.Request()
            req.x = float(x)
            req.y = float(y)
            req.theta = 0.0
            self.teleport.call_async(req)
            rclpy.spin_once(self, timeout_sec=0.5)

def main():
    rclpy.init()
    node = RRTPlanner()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
