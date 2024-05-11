#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from time import time
from math import sin
from numpy import linspace , inf

class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)
        self.bridge = CvBridge()
        self.width = 0.1  # half of the defined width (1 meter)
        self.extent = self.width / 2.0
        
        # Initialize publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Twist message for robot control
        self.twist = Twist()
        self.min_angular_vel = -0.3
        self.max_angular_vel = 0.3

        # Define the turning angle for each increment
        self.turn_angle = np.pi / 2  # 90 degrees in radians

        # Define the turning velocity for 90-degree turns
        self.turning_vel_90 = 0.5

        # Flag to indicate whether the robot is currently turning
        self.is_turning = False

        # Flag to indicate whether the robot is currently following the line
        self.is_following_line = False
        
        # Obstacle detection parameters
        self.obstacle_detected = False
        self.obstacle_distance_threshold = 0.1  # meters
        
        # Line following parameters
        self.last_line_detection_time = time()
        self.max_time_without_line_detection = 1.5  # seconds


    
    def move_forward(self, error):
        self.twist.linear.x = 0.1
        self.twist.angular.z = -float(error) / 100
        self.twist.angular.z = max(self.min_angular_vel, min(self.max_angular_vel, self.twist.angular.z))
        self.cmd_vel_pub.publish(self.twist)
        self.is_turning = False
        self.is_following_line = True
        print("Robot keeps going forward")

    def stop_robot(self):
        """ Stop all robot motion. """
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        self.is_following_line = False



    def turn(self, direction):
        # Set the angular velocity for turning
        self.twist.angular.z = self.turning_vel_90 if direction == 'left' else -self.turning_vel_90
        self.is_turning = True
        print("Robot turning", direction)

    def avoid_obstacle(self):
        # Stop the robot

        # Perform obstacle avoidance
        print("Obstacle detected ahead!")
        # In this simplified version, we'll just turn left
        self.turn('right')

    def scan_callback(self, msg):
        # Process LiDAR scans to check for obstacles directly in front of the robot
        angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        points = [r * sin(theta) if (theta < -2.5 or theta > 2.5) else inf for r, theta in zip(msg.ranges, angles)]
        filtered_ranges =[r if abs(y) < self.extent else inf for r, y in zip(msg.ranges, points)]
        
        if min(filtered_ranges) <= 0.2:
            self.obstacle_detected = True
            self.is_following_line = False
            self.twist.linear.x = 0
            self.twist.angular.z = -0.3  # Adjust angular velocity for obstacle avoidance
            self.cmd_vel_pub.publish(self.twist)
        else:
            self.obstacle_detected = False
            self.is_following_line = True
              # seconds
            
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Crop the lower part of the image to focus only on the floor
        height, width, _ = cv_image.shape
        crop_height = height // 2  # Crop from the middle of the image downwards
        cropped_image = cv_image[crop_height:, :]

        # Convert cropped image from BGR to HSV
        hsv = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for white color in HSV
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])

        # Threshold the HSV image to get only white colors
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Calculate the center of mass of the white region
        M = cv2.moments(mask)
        if self.is_following_line:
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # Calculate error from the center of the image
                error = cx - cropped_image.shape[1] / 2

                self.move_forward(error)
                # Update the timestamp when the line was last detected
                self.last_line_detection_time = time()
            else:
                
                # Check if it's been too long since the last line detection
                if time() - self.last_line_detection_time > self.max_time_without_line_detection:
                    self.stop_robot()
                    # Avoid obstacle
                    self.avoid_obstacle()

        # Publish velocity command
        self.cmd_vel_pub.publish(self.twist)
        cv2.waitKey(3)

if __name__ == '__main__':
    try:
        LineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Line follower node terminated.")
