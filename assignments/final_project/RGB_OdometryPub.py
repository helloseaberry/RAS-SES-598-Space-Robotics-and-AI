import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
import numpy as np
import cv2
from cv_bridge import CvBridge
import math
from transforms3d.euler import euler2quat  # replaces tf_transformations

class FakeRGBDOdometryPublisher(Node):
    def __init__(self):
        super().__init__('fake_rgbd_odometry_publisher')
        self.bridge = CvBridge()
        self.rgb_pub = self.create_publisher(Image, '/rgb_camera/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/depth_camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/rgb_camera/camera_info', 10)
        self.odom_pub = self.create_publisher(Odometry, '/mavros/odometry/in', 10)
        self.timer = self.create_timer(0.1, self.publish_fake_data)
        self.t = 0.0

    def publish_fake_data(self):
        # Create blank images
        rgb_image = np.full((480, 640, 3), 128, dtype=np.uint8)
        depth_image = np.full((480, 640), 3.0, dtype=np.float32)  # Far background

        # Draw simulated "obstacles" in RGB
        cv2.rectangle(rgb_image, (100, 100), (180, 200), (0, 0, 255), -1)  # Red box
        cv2.rectangle(rgb_image, (300, 250), (400, 350), (0, 255, 0), -1)  # Green box
        cv2.rectangle(rgb_image, (500, 100), (580, 200), (255, 0, 0), -1)  # Blue box

        # Simulate depth for obstacles (closer than background)
       
        depth_image = np.full((480, 640), 3.0, dtype=np.float32)  # Background = 3 meters
        # Fake "closer" obstacles:
        depth_image[100:200, 100:180] = 0.8
        depth_image[250:350, 300:400] = 1.0
        depth_image[100:200, 500:580] = 0.5


        # Convert to ROS Image messages
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='32FC1')

        timestamp = self.get_clock().now().to_msg()
        rgb_msg.header.stamp = timestamp
        rgb_msg.header.frame_id = 'camera_link'
        depth_msg.header = rgb_msg.header

        # Camera Info
        cam_info = CameraInfo()
        cam_info.header = rgb_msg.header
        cam_info.height = 480
        cam_info.width = 640
        cam_info.k = [525.0, 0.0, 319.5,
                    0.0, 525.0, 239.5,
                    0.0, 0.0, 1.0]
        cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        cam_info.r = [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]
        cam_info.p = [525.0, 0.0, 319.5, 0.0,
                    0.0, 525.0, 239.5, 0.0,
                    0.0, 0.0, 1.0, 0.0]
        cam_info.distortion_model = "plumb_bob"


        # Simulated Odometry
        odom = Odometry()
        odom.header = rgb_msg.header
        odom.child_frame_id = 'base_link'
        x = 0.5 * math.cos(self.t)
        y = 0.5 * math.sin(self.t)
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        q_wxyz = euler2quat(0, 0, self.t)
        odom.pose.pose.orientation.w = q_wxyz[0]
        odom.pose.pose.orientation.x = q_wxyz[1]
        odom.pose.pose.orientation.y = q_wxyz[2]
        odom.pose.pose.orientation.z = q_wxyz[3]

        # Publish
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.info_pub.publish(cam_info)
        self.odom_pub.publish(odom)

        self.t += 0.05

def main(args=None):
    rclpy.init(args=args)
    node = FakeRGBDOdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
