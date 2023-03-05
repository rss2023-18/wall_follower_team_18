#!/usr/bin/env python2

# from distutils.log import error
import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *

class PID:
    def _init_(self, Kp, Ki, Kd):
        self.previous_time = rospy.get_rostime()
        self.proportional_gain = Kp
        self.integral_gain = Ki
        self.derivative_gain = Kd
        self.alpha = 0.2
        self.integrator_bounds = (-15, 15)

        self.current_error = 0
        self.previous_error = 0
        self.error_derivative = 0
        self.error_integral = 0
      

    def control(self, target, actual):
        current_time = rospy.get_rostime()
        self.previous_time = current_time

        self.current_error = target - actual

        self.error_derivative = self.alpha * (self.current_error - self.previous_error) + (1 - self.alpha) * self.error_derivative
        
        # Update integral error
        error_integral_temp = self.error_integral + (self.current_error * (current_time - self.previous_time).to_sec())
        if error_integral_temp < self.integrator_bounds[0]:
            self.error_integral = self.integrator_bounds[0]
        elif error_integral_temp > self.integrator_bounds[1]:
            self.error_integral = self.integrator_bounds[1]
        else:
            self.error_integral = error_integral_temp
        
        self.previous_error = self.current_error

        # Calculate control output
        return self.proportional_gain * self.current_error + self.derivative_gain * self.error_derivative + self.integral_gain * self.error_integral

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
   

    def _init_(self):
        self.distance_controller = PID(1, 0, 0)
        self.angle_controller = PID(1, 0, 0)       
        self.ackermann_publisher = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.wall_publisher = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=10)
        self.point_publisher = rospy.Publisher(self.POINT_TOPIC, Marker, queue_size=10)
        self.lidar_subscriber = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)      
        self.angle_wide = -0.6*np.pi
        self.angle_front = np.pi / 18
 
    def get_distance_and_angle(self, data, min_angle, max_angle):
        
        def get_angle_index(angle):
            return int((angle - data.angle_min) / data.angle_increment)

        # Subset the range and angle data to within the desired slice
        range_subset = data.ranges[get_angle_index(min_angle):get_angle_index(max_angle)]
        
        angle_subset = [min_angle + i * data.angle_increment for i in range(len(range_subset))]

        # Convert range and angle data to Cartesian coordinates in robot frame
        points_x = []
        points_y = []
        for i in range(len(range_subset)):
            points_x.append(range_subset[i] * np.cos(angle_subset[i]))
            points_y.append(range_subset[i] * np.sin(angle_subset[i]))

        # Assign weights to each point based on its range, where farther points are weighted less
        weights = [1.0/(range_subset[i]**3) for i in range(len(range_subset))]

        # Fit a line to the Cartesian points, with weighting based on range
        (m, b) = np.polyfit(points_x, points_y, 1, w=weights)

        return (b*np.cos(np.arctan(m)), np.arctan(m)) # actual angle, actual distance


    def callback(self, data):

        if self.SIDE == -1: #right
            min_angle = self.angle_wide
            max_angle = self.angle_front
        else:
            min_angle = -self.angle_front
            max_angle = -self.angle_wide
        
        (actual_distance, actual_angle) = self.get_distance_and_angle(data, min_angle, max_angle)
        
        if self.SIDE == -1: #right
            target_angle = self.distance_controller.control(-self.DESIRED_DISTANCE, actual_distance)
        else:
            target_angle = self.distance_controller.control(self.DESIRED_DISTANCE, actual_distance)
        
        if target_angle > np.pi/3:
            target_angle = np.pi/3
        elif target_angle < -np.pi/3:
            target_angle = -np.pi/3
        else:
            target_angle = target_angle
        
        drive_angle = self.angle_controller.control(target_angle, actual_angle)
        
        self.send_drive_command(-drive_angle)


    def send_drive_command(self, angle):

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.get_rostime()
        drive_msg.drive.acceleration = 0
        drive_msg.drive.speed = self.VELOCITY

        drive_msg.drive.steering_angle_velocity = 0
        drive_msg.drive.steering_angle = angle

        self.ackermann_publisher.publish(drive_msg)


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()    