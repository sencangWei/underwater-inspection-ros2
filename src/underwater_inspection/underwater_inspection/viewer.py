"""Presentation runs separately so a GUI cannot stall mission callbacks."""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    rclpy.init()
    node = Node('inspection_dashboard_viewer')
    bridge = CvBridge()
    title = 'Underwater inspection - LIVE sensors'
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 960, 540)
    cv2.imshow(title, np.zeros((720, 1280, 3), np.uint8))
    cv2.waitKey(1)

    def show(msg):
        cv2.imshow(title, bridge.imgmsg_to_cv2(msg, 'bgr8'))
        cv2.waitKey(1)

    node.create_subscription(Image, '/inspection/dashboard', show, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
