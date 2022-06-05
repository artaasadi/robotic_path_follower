#!/usr/bin/python3

from cmath import rect
from matplotlib import pyplot as plt
import numpy as np
import rospy
import tf
import math

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

from math import atan2, pi, radians, sqrt

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)

        # getting specified parameters
        self.linear_speed = rospy.get_param("/controller/linear_speed") # m/s
        self.angular_speed = rospy.get_param("/controller/angular_speed") # rad/s
        self.goal_angle = radians(rospy.get_param("/controller/goal_angle")) # rad
        self.stop_distance = rospy.get_param("/controller/stop_distance") # m
        self.epsilon = rospy.get_param("/controller/epsilon")
        self.distanceThreshold = 0.3
        self.twist = Twist()
        self.errors = []
        
        # defining the states of our robot
        self.GO, self.ROTATE = 0, 1
        self.state = self.GO 

        self.kp_distance = 10
        self.ki_distance = 0
        self.kd_distance = 0.5

        self.kp_angle = 3
        self.ki_angle = 0.03
        self.kd_angle = 0.05
        

    def rectangle(self):
        rectangle = []
        x1 = np.linspace(2, -2 , 20)
        rectangle = [*rectangle, *[[i, 3.0] for i in x1]]
        
        x2 = np.linspace(-2, 2 , 20)
        rectangle = [*rectangle, *[[i, -3.0] for i in x2]]

        y1 = np.linspace(3, -3 , 10)
        rectangle = [*rectangle, *[[2.0, i] for i in y1]]

       
        y2 = np.linspace(-3, 3 , 10)
        rectangle = [*rectangle, *[[-2.0, i] for i in y2]]
        
        self.path = rectangle

    def archimedean_spiral(self):
        archimedean_spiral = []
        growth_factor = 0.1
        for i in range(400):
            t = i / 20 * math.pi
            dx = (1 + growth_factor * t) * math.cos(t)
            dy = (1 + growth_factor * t) * math.sin(t)
            archimedean_spiral.append([dx,dy])
        
        self.path = archimedean_spiral

    def logarithmic_spiral(self) :
        logarithmic_spiral = [] 
        a = 0.17
        k = math.tan(a)

        for i in range(150):
            t = i / 20 * math.pi
            dx = a * math.exp(k * t) * math.cos(t)
            dy = a * math.exp(k * t) * math.sin(t)
            logarithmic_spiral.append([dx,dy])
        
        self.path = logarithmic_spiral

    def hasht_zel(self):
        hasht_zel = []
        X1 = np.linspace(-1, 1 , 10)
        hasht_zel = [*hasht_zel, *[[i, 3.0] for i in X1]]


        X2 = np.linspace(1, 1 + 2**(1/2) , 10)
        Y2 = - (2**(1/2)) * (X2 - 1) + 3
        hasht_zel = [*hasht_zel, *[(i, j) for (i, j) in zip(X2, Y2)]]

        Y3 = np.linspace(1, -1 , 10)
        X3 = np.array([1 + 2**(1/2)]*10)
        hasht_zel = [*hasht_zel, *[(i, j) for (i, j) in zip(X3, Y3)]]

        X4 = np.linspace(1 + 2**(1/2), 1, 10)
        Y4 = (2**(1/2)) * (X4 - 1 - 2**(1/2)) -1 
        hasht_zel = [*hasht_zel, *[(i, j) for (i, j) in zip(X4, Y4)]]

        X5 = np.linspace(1, -1 , 10)
        hasht_zel = [*hasht_zel, *[[i, -3.0] for i in X5]]

        X6 = np.linspace(-1, -1 - 2**(1/2) , 10)
        Y6 = - (2**(1/2)) * (X6 + 1) - 3 
        hasht_zel = [*hasht_zel, *[(i, j) for (i, j) in zip(X6, Y6)]]


        Y7 = np.linspace(-1, 1 , 10)
        X7 = np.array([- 1 - 2**(1/2)]*10)
        hasht_zel = [*hasht_zel, *[(i, j) for (i, j) in zip(X7, Y7)]]


        X8 = np.linspace(-1 - 2**(1/2), -1, 10)
        Y8 = (2**(1/2)) * (X8 + 1 + 2**(1/2)) + 1
        hasht_zel = [*hasht_zel, *[(i, j) for (i, j) in zip(X8, Y8)]]


        self.path = hasht_zel

    def circles(self):
        circles = []
        X1 = np.linspace(-6., -2 , 50)
        circles = [*circles, *[[i, 0.0] for i in X1]]

        x_dim, y_dim = 2,2
        t = np.linspace(np.pi, 0, 100)
        circles = [*circles, *[[x_dim*np.cos(i), y_dim*np.sin(i)] for i in t]]

        X3 = np.linspace(2, 6 , 50)
        circles = [*circles, *[[i, 0.0] for i in X3]]

        x_dim, y_dim = 6,6
        t = np.linspace(np.pi*2, np.pi, 200)
        circles = [*circles, *[[x_dim*np.cos(i), y_dim*np.sin(i)] for i in t]]

        self.path = circles

    # heading of the robot 
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        _, _, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        return yaw        

    def run(self):

        #self.rectangle()
        #self.archimedean_spiral()
        #self.logarithmic_spiral()
        #self.hasht_zel()
        #self.circles()
        shape = rospy.get_param("/controller/shape")
        if shape == "rectangle" :
            self.rectangle()
        elif shape == "archimedean_spiral" :
            self.archimedean_spiral()
        elif shape == "logarithmic_spiral" :
            self.logarithmic_spiral()
        elif shape == "hasht_zel" :
            self.hasht_zel()
        elif shape == "circles" :
            self.circles()
        else :
            self.rectangle()

        while not rospy.is_shutdown():
            last_angle = 0
            for [x, y] in self.path:
                self.cmd_publisher.publish(Twist())

                msg = rospy.wait_for_message("/odom" , Odometry) 
                curr_x = msg.pose.pose.position.x
                curr_y = msg.pose.pose.position.y

                delta_x = x - curr_x
                delta_y = y - curr_y

                distance = sqrt(delta_x**2 + delta_y**2)

                total_distance = 0
                total_angle = 0
                previous_distance = 0

                while distance > 0.1:
                    
                    current_angle = self.get_heading()
                    msg = rospy.wait_for_message("/odom" , Odometry) 
                    curr_x = msg.pose.pose.position.x
                    curr_y = msg.pose.pose.position.y

                    self.errors.append(sqrt(min([((curr_x - i) ** 2 + (curr_y - j) ** 2) for [i, j] in self.path])))

                    delta_x = x - curr_x
                    delta_y = y - curr_y

                    path_angle = atan2(delta_y , delta_x) 

                    if last_angle > pi-0.1 and current_angle <= 0:
                        current_angle = 2*pi + current_angle
                    elif last_angle < -pi+0.1 and current_angle > 0:
                        current_angle = -2*pi + current_angle
                        
                    if path_angle < -pi/4 or path_angle > pi/4:
                        if y < 0 and curr_y < y:
                            path_angle = -2*pi + path_angle
                        elif y >= 0 and curr_y > y:
                            path_angle = 2*pi + path_angle


                    distance = sqrt(delta_x**2 + delta_y**2)

                    diff_distance = distance - previous_distance

                    control_signal_distance = self.kp_distance*distance + self.ki_distance*total_distance + self.kd_distance*diff_distance

                    control_signal_angle = self.kp_angle*(path_angle - current_angle)

                    

                    self.twist.angular.z = (control_signal_angle)

                    self.twist.linear.x = min(control_signal_distance,0.1)

                    if self.twist.angular.z > 0:
                        self.twist.angular.z = min(self.twist.angular.z, 1.5)
                    else:
                        self.twist.angular.z = max(self.twist.angular.z, -1.5)
                    
                    last_angle = current_angle
                    self.cmd_publisher.publish(self.twist)

                    total_distance += distance
                    previous_distance = distance
                    total_angle += path_angle
            self.cmd_publisher.publish(Twist())
            plt.plot(list(range(1, len(self.errors) + 1)), self.errors)
            plt.show()
            break

if __name__ == "__main__":
    controller = Controller()
    controller.run()