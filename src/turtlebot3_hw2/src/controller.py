#!/usr/bin/python3

from cmath import sqrt
from multiprocessing import set_forkserver_preload
import rospy, tf
#from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import radians
import numpy as np
import matplotlib.pyplot as plt


X1 = np.linspace(0, 6 , 100)
Y1 = np.array([4]*100)

Y2 = np.linspace(4, 0, 100)
X2 = np.array([6]*100)

X3 = np.linspace(6, 0, 100)
Y3 = np.array([0]*100)

Y4 = np.linspace(0, 4, 100)
X4 = np.array([0]*100)

class Controller:
    def __init__(self) -> None:
        rospy.init_node("controller", anonymous=False)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        # getting specified parameters
        self.linear_speed = rospy.get_param("/controller/linear_speed") # m/s
        self.angular_speed = rospy.get_param("/controller/angular_speed") # rad/s
        self.goal_angle = radians(rospy.get_param("/controller/goal_angle")) # rad
        self.epsilon = rospy.get_param("/controller/epsilon")
        self.length = 6 #rospy.get_param("/controller/length")
        self.width = 4 #rospy.get_param("/controller/width")
        # defining the states of our robot
        self.GO = 1
        self.IN_LENGTH = 1
        self.steps = []
        self.errors = []
        self.counter = 0

    
    
    def rotate(self):
        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)
        
        msg = rospy.wait_for_message("/odom" , Odometry) 
        orientation = msg.pose.pose.orientation
        _, _, prev_angle = tf.transformations.euler_from_quaternion((orientation.x ,orientation.y ,orientation.z ,orientation.w))
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.cmd_publisher.publish(twist)
        current_angle = prev_angle      
        while self.goal_angle >= abs(prev_angle - current_angle):
            msg = rospy.wait_for_message("/odom" , Odometry)
            orientation = msg.pose.pose.orientation
            _, _, temp_angle = tf.transformations.euler_from_quaternion((orientation.x ,orientation.y ,orientation.z ,orientation.w))
            if temp_angle > 0:
                current_angle -= abs(abs(temp_angle) - abs(current_angle))
            else :
                current_angle = temp_angle
        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)            
    
    
    def move(self):
        traveled = 0
        steps = 0
        if self.IN_LENGTH == 1:
                steps = self.length
                self.IN_LENGTH = 0
        else:
                steps = self.width
                self.IN_LENGTH = 1
        msg = rospy.wait_for_message("/odom" , Odometry) 
        start_x = msg.pose.pose.position.x
        start_y = msg.pose.pose.position.y
        while steps > traveled:
            twist = Twist()
            twist.linear.x = self.linear_speed
            twist.angular.z = 0
            self.cmd_publisher.publish(twist)
            msg = rospy.wait_for_message("/odom" , Odometry) 
            y_pos= msg.pose.pose.position.y
            x_pos= msg.pose.pose.position.x
            self.errors.append(min(min(abs(X1 - x_pos) + abs(Y1 - y_pos)), min(abs(X2 - x_pos) + abs(Y2 - y_pos)), min(abs(X3 - x_pos) + abs(Y3 - y_pos)), min(abs(X4 - x_pos) + abs(Y4 - y_pos))))
            traveled = abs(y_pos - start_y) + abs(x_pos - start_x)  
    

    def run(self):
        while not rospy.is_shutdown():
            self.move()
            self.counter += 1
            self.rotate()
            if self.counter == 4:
                break
        plt.plot(list(range(1, len(self.errors) + 1)), self.errors)
        plt.show()
              

if __name__ == "__main__":
    controller = Controller()
    controller.run()