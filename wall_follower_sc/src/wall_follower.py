#!/usr/bin/env python2

import numpy as np
import math
from sklearn.linear_model import LinearRegression

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *

def angle_to_index(angle, mini, inc):
    return int((angle-mini)/inc)

def index_to_angle(ind, mini, inc):
    return mini+inc*ind

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    WALL_TOPIC = "/wall"
    FRONT_WALL = "/front_wall"

    def __init__(self):
        # TODO:
        # Initialize your publishers and
        # subscribers here
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped)
        self.sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan)

        self.line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=1)
        self.front_line = rospy.Publisher(self.FRONT_WALL, Marker, queue_size=1)

        rate = rospy.Rate(20)
        self.listener()
    # TODO:
    # Write your callback functions here.
    def callback(self, msg):
        #all (x,y) coords calculated from d*cos(angle) and d*sin(angle)
        x_all = np.array([msg.ranges[i]*math.cos(msg.angle_min+msg.angle_increment*i) for i in range(len(msg.ranges))])
        y_all = np.array([msg.ranges[i]*math.sin(msg.angle_min+msg.angle_increment*i) for i in range(len(msg.ranges))])
        # feedback amounts
        kp, kd, ki, dt = (6, 3, 0, 0.05)
        if self.SIDE == -1:
            x_right = np.array(x_all[angle_to_index(-1.309, msg.angle_min, msg.angle_increment):angle_to_index(0, msg.angle_min, msg.angle_increment)]).reshape(-1,1)
            y_right = np.array(y_all[angle_to_index(-1.309, msg.angle_min, msg.angle_increment):angle_to_index(0, msg.angle_min, msg.angle_increment)])

            #front
            front_right_x = np.array(x_all[angle_to_index(0, msg.angle_min, msg.angle_increment):angle_to_index(0.39, msg.angle_min, msg.angle_increment)]).reshape(-1,1)
            front_right_y = np.array(y_all[angle_to_index(0, msg.angle_min, msg.angle_increment):angle_to_index(0.39, msg.angle_min, msg.angle_increment)])

            front_wall = LinearRegression().fit(front_right_x, front_right_y)
            front_wall = LinearRegression().fit(front_right_x, front_right_y)
            f_r_predict = front_wall.predict(front_right_x)
            m_front = front_wall.coef_
            b_front = front_wall.intercept_
            front_x = (-m_front*b_front)/(1+m_front**2)
            front_y = b_front/(1+m_front**2)
            dist_front = float(math.sqrt(front_x**2+front_y**2))

            r_wall = LinearRegression().fit(x_right, y_right)
            y_r_predict = r_wall.predict(x_right)
            VisualizationTools.plot_line(x_right, y_r_predict, self.line_pub, frame="/laser")
            #slope and intercept of side linear regression
            m = r_wall.coef_
            b = r_wall.intercept_
            #(x,y) coord for dist from wall
            x = (-m*b)/(1+m**2)
            y = b/(1+m**2)

            # dist_from_wall = math.sqrt(x**2+y**2)
            dist_from_wall = float(abs(y))

            print('dist=', dist_from_wall)

            #error e(t)
            error = self.DESIRED_DISTANCE - dist_from_wall
            print('error=', error)
            
            #looking for angle change
            a0 = kp + kd*dt + ki/dt
            a1 = -kp - 2*kd/dt
            a2 = kd/dt
            #current value of actuator
            output = 0
            #e(t-2)
            e2 = 0
            #e(t-1)
            e1 = 0
            #e(t)
            e0 = 0
            
            e2 = e1
            e1 = e0
            e0 = self.DESIRED_DISTANCE - dist_from_wall
            output += a0 * e0 + a1*e1 + a2*e2
            if abs(self.DESIRED_DISTANCE - dist_front) < 0.3:
                VisualizationTools.plot_line(front_right_x, f_r_predict, self.front_line, frame="/laser")
                output = 0.34
            print('output:', output)

        else:
            x_left = np.array(x_all[angle_to_index(0, msg.angle_min, msg.angle_increment):angle_to_index(1.309, msg.angle_min, msg.angle_increment)]).reshape(-1,1)
            y_left = np.array(y_all[angle_to_index(0, msg.angle_min, msg.angle_increment):angle_to_index(1.309, msg.angle_min, msg.angle_increment)])

            #front angles
            front_left_x = np.array(x_all[angle_to_index(-0.39, msg.angle_min, msg.angle_increment):angle_to_index(0, msg.angle_min, msg.angle_increment)]).reshape(-1,1)
            front_left_y = np.array(y_all[angle_to_index(-0.39, msg.angle_min, msg.angle_increment):angle_to_index(0, msg.angle_min, msg.angle_increment)])
            #front wall look
            front_wall = LinearRegression().fit(front_left_x, front_left_y)
            f_l_predict = front_wall.predict(front_left_x)
            m_front = front_wall.coef_
            b_front = front_wall.intercept_
            front_x = (-m_front*b_front)/(1+m_front**2)
            front_y = b_front/(1+m_front**2)
            dist_front = float(math.sqrt(front_x**2+front_y**2))

            l_wall = LinearRegression().fit(x_left, y_left)
            y_l_predict = l_wall.predict(x_left)
            VisualizationTools.plot_line(x_left, -y_l_predict, self.line_pub, frame="/laser")
            m = l_wall.coef_
            b = l_wall.intercept_
            #perpendicular intercept between car origin and wall
            x = (-m*b)/(1+m**2)
            y = b/(1+m**2)

            # dist_from_wall = math.sqrt(x**2+y**2)
            dist_from_wall = float(abs(y))

            print('dist=', dist_from_wall)

            #error e(t)
            error = self.DESIRED_DISTANCE - dist_from_wall
            print('error=', error)
            #looking for angle change
            a0 = kp + kd*dt + ki/dt
            a1 = -kp - 2*kd/dt
            a2 = kd/dt
            #current value of actuator
            output = 0
            #e(t-2)
            e2 = 0
            #e(t-1)
            e1 = 0
            #e(t)
            e0 = 0
            
            e2 = e1
            e1 = e0
            e0 = dist_from_wall - self.DESIRED_DISTANCE
            
            output += a0 * e0 + a1*e1 + a2*e2
            if abs(dist_front - self.DESRIED_DISTANCE) < 0.3:
                VisualizationTools.plot_line(front_left_x, f_l_predict, self.front_line, frame="/laser")
                output = -0.34
            print('output:', output)            
    
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.header.frame_id = 'base_link'
        ack_msg.drive.steering_angle = output
        ack_msg.drive.speed = self.VELOCITY
        self.pub.publish(ack_msg)
        pass

    def listener(self):
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        rospy.spin()
        pass

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
