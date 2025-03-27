#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
from scipy import linalg

class CartPoleLQRController(Node):
    def __init__(self):
        super().__init__('cart_pole_lqr_controller')
        
        # System parameters
        self.M = 1.0
        self.m = 1.0
        self.L = 1.0
        self.g = 9.81
        
        self.A = np.array([
            [0, 1, 0, 0],
            [0, 0, (self.m * self.g) / self.M, 0],
            [0, 0, 0, 1],
            [0, 0, ((self.M + self.m) * self.g) / (self.M * self.L), 0]
        ])
        self.B = np.array([[0], [1/self.M], [0], [-1/(self.M * self.L)]])
        
        # LQR cost matrices
        self.Q = np.diag([10.0, 2.0, 50.0, 5.0]) # State cost
        self.R = np.array([[0.5]]) # Control cost
        # Compute LQR gain matrix
        self.K = self.compute_lqr_gain()
        self.get_logger().info(f'LQR Gain Matrix: {self.K}')
        # Initialize state estimate
        self.x = np.zeros((4, 1))
        self.state_initialized = False
        self.last_control = 0.0
        self.control_count = 0
        # Create publishers and subscribers        
        self.cart_cmd_pub = self.create_publisher(
            Float64, 
            '/model/cart_pole/joint/cart_to_base/cmd_force', 
            10
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/world/empty/model/cart_pole/joint_state',
            self.joint_state_callback,
            10
        )
        self.timer = self.create_timer(0.01, self.control_loop)
        self.get_logger().info('Cart-Pole LQR Controller initialized')

    def compute_lqr_gain(self):
        P = linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ self.B.T @ P
        return K

    def joint_state_callback(self, msg):
        """Update state estimate from joint states."""
        self.get_logger().info(f"Received Joint State: names={msg.name}, positions={msg.position}, velocities={msg.velocity}")

        try:
            # Check if required joints are present
            if 'cart_to_base' not in msg.name or 'pole_joint' not in msg.name:
                self.get_logger().warn(f"Missing required joints in {msg.name}")
                return

            cart_idx = msg.name.index('cart_to_base')
            pole_idx = msg.name.index('pole_joint')

            # Ensure position and velocity data exist
            if len(msg.position) <= max(cart_idx, pole_idx) or len(msg.velocity) <= max(cart_idx, pole_idx):
                self.get_logger().warn("Insufficient position/velocity data!")
                return

            # Update state vector
            self.x = np.array([
                [msg.position[cart_idx]],  # Cart position
                [msg.velocity[cart_idx]],  # Cart velocity
                [msg.position[pole_idx]],  # Pole angle
                [msg.velocity[pole_idx]]   # Pole angular velocity
            ])
            
            if not self.state_initialized:
                self.get_logger().info(f'Initial state: cart_pos={self.x[0,0]:.3f}, cart_vel={self.x[1,0]:.3f}, pole_angle={self.x[2,0]:.3f}, pole_vel={self.x[3,0]:.3f}')
                self.state_initialized = True

        except Exception as e:
            self.get_logger().warn(f'Failed to process joint states: {e}')

    def control_loop(self):
        if not self.state_initialized:
            self.get_logger().warn('State not initialized yet')
            return
        # Compute control input u = -Kx
        u = -self.K @ self.x
        force = float(u[0])
        # Log control input periodically
        if abs(force - self.last_control) > 0.1 or self.control_count % 100 == 0:
            self.get_logger().info(f'State: {self.x.T}, Control force: {force:.3f}N')
        # Publish control command        
        msg = Float64()
        msg.data = force
        self.cart_cmd_pub.publish(msg)
        
        self.last_control = force
        self.control_count += 1

def main(args=None):
    rclpy.init(args=args)
    controller = CartPoleLQRController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()