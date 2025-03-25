#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import time

class CartPolePlotter(Node):
    def __init__(self):
        super().__init__('cart_pole_plotter')

        self.time_data = []
        self.cart_pos = []
        self.pole_angle = []
        self.control_force = []

        self.start_time = time.time()

        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.create_subscription(Float64, '/model/cart_pole/joint/cart_to_base/cmd_force', self.force_callback, 10)

        self.timer = self.create_timer(0.1, self.update_plot)

        plt.ion()
        self.fig, self.axs = plt.subplots(3, 1, figsize=(8, 8))

    def joint_callback(self, msg):
        now = time.time() - self.start_time
        self.time_data.append(now)
        self.cart_pos.append(msg.position[0])
        self.pole_angle.append(msg.position[1])

    def force_callback(self, msg):
        self.control_force.append(msg.data)

    def update_plot(self):
        if not self.time_data:
            return
        self.axs[0].cla()
        self.axs[1].cla()
        self.axs[2].cla()

        self.axs[0].plot(self.time_data, self.cart_pos, label='Cart Position')
        self.axs[1].plot(self.time_data, self.pole_angle, label='Pole Angle', color='green')
        self.axs[2].plot(self.time_data[:len(self.control_force)], self.control_force, label='Control Force', color='red')

        self.axs[0].set_ylabel("Cart Position (m)")
        self.axs[1].set_ylabel("Pole Angle (rad)")
        self.axs[2].set_ylabel("Force (N)")
        self.axs[2].set_xlabel("Time (s)")

        for ax in self.axs:
            ax.legend()
            ax.grid(True)

        plt.tight_layout()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = CartPolePlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
