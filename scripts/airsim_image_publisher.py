#!/usr/bin/env python3
"""Publish AirSim camera images to ROS for ORB-SLAM2."""
import airsim
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


def main():
    rospy.init_node('airsim_image_publisher')
    pub = rospy.Publisher('/airsim/image_raw', Image, queue_size=1)
    bridge = CvBridge()
    client = airsim.MultirotorClient()
    client.confirmConnection()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        resp = client.simGetImage('0', airsim.ImageType.Scene)
        if resp is None:
            rospy.logwarn('No image received from AirSim')
            rate.sleep()
            continue
        img1d = np.frombuffer(resp, dtype=np.uint8)
        img_bgr = cv2.imdecode(img1d, cv2.IMREAD_COLOR)
        if img_bgr is None:
            rospy.logwarn('Failed to decode image')
            rate.sleep()
            continue
        msg = bridge.cv2_to_imgmsg(img_bgr, encoding='bgr8')
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    main()
