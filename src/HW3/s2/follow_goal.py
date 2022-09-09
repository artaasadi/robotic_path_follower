#!/usr/bin/python3

import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import atan2
from cmath import sqrt

import matplotlib.pyplot as plt

class PIDController():


    def __init__(self):
        
        rospy.init_node('ros_tutorial', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        
        self.goal_x = 3
        self.goal_y = -1

        self.currentX = 0
        self.currentY = 0

        self.k_i = 0.08
        self.k_p = 0.6
        self.k_d = 6
        
        self.dt = 0.005
        self.v = 0.06
        self.D = 0.7
        rate = 1/self.dt
        
        self.r = rospy.Rate(rate)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.errs = []


    def distance_from_wall(self):
        laser_data = rospy.wait_for_message("/scan" , LaserScan)
        rng = laser_data.ranges[10:190]
        d = min(rng)
        return d
    
    def front_distance_from_wall(self):
        laser_data = rospy.wait_for_message("/scan" , LaserScan)
        rng1 = laser_data.ranges[0:10]
        rng2 = laser_data.ranges[350:359]
        d = min(min(rng1),min(rng2))
        return d

    def distance_from_goal(self):
        msg = rospy.wait_for_message("/odom" , Odometry) 
        self.currentX = msg.pose.pose.position.x
        self.currentY = msg.pose.pose.position.y
        deltaX = self.currentX - self.goal_x
        deltaY = self.currentY - self.goal_y
        d = sqrt(deltaX**2 + deltaY**2)
        return d

    def angle_to_goal(self):
        inc_x = self.goal_x - self.currentX
        inc_y = self.goal_y - self.currentY
        angle = atan2(inc_y, inc_x)
        return angle

    # heading of the robot 
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        return yaw   

    
    def go_to_goal(self):

        # while not rospy.is_shutdown():
            
            move_cmd = Twist()
            angle = self.angle_to_goal()
            # rospy.loginfo(f"GO TO GOAL")
            # rospy.loginfo(f"@@@@@@@@@@@angle : {angle}")
            delta = angle - self.get_heading()
            # rospy.loginfo(f"$$$$$$$$$$delta : {delta}")
            # front_d = self.front_distance_from_wall()
            # rospy.loginfo(f"@@@@@@@@@@@front_d : {front_d}")

            if abs(delta) > 0.2:
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.3
                # rospy.loginfo(f"ROTATE")
            else:
                move_cmd.linear.x = self.v
                move_cmd.angular.z = 0.0
                # rospy.loginfo(f"GOOOOOOOOO")

            self.cmd_vel.publish(move_cmd)
            # self.r.sleep() 


    def follow_wall(self):

        # rospy.loginfo(f"WALL")
        
        d = self.distance_from_wall()    
        sum_i_theta = 0
        prev_theta_error = 0
            
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.v

        front_d = self.front_distance_from_wall()
        if front_d <= (self.D + 0.1):
            front_d = self.front_distance_from_wall()
            # rospy.loginfo(f"$$$$$$$$$$ : {front_d}")
            # rospy.loginfo(f"WALL")
            twist = Twist()
            twist.angular.z = -0.2
            # move_cmd.linear.x = 0.0
            self.cmd_vel.publish(twist)
            # twist = Twist()
            # move_cmd.linear.x = self.v
            # move_cmd.angular.z = 0.0
            # self.cmd_vel.publish(twist)
            
        else:
            self.cmd_vel.publish(move_cmd)
            # rospy.loginfo(f"OK")
            err = d - self.D
            self.errs.append(err)
            sum_i_theta += err * self.dt
            
            P = self.k_p * err
            I = self.k_i * sum_i_theta
            D = self.k_d * (err - prev_theta_error)

            # rospy.loginfo(f"P : {P} I : {I} D : {D}")
            move_cmd.angular.z = P + I + D 
            prev_theta_error = err
            move_cmd.linear.x = self.v                
            # rospy.loginfo(f"error : {err} speed : {move_cmd.linear.x} theta : {move_cmd.angular.z}")
             

    def robot_action(self):
        goal_d = self.distance_from_goal()
        rospy.loginfo(f"#GOAL : {goal_d}")
        while not rospy.is_shutdown() :
            goal_d = self.distance_from_goal()
            if goal_d.real > 0.35:
                front_d = self.front_distance_from_wall()
                # rospy.loginfo(f"@@@@@@@@@@@front_d : {front_d-self.D}")
                d = self.distance_from_wall()
                # rospy.loginfo(f"@@@@@@@@@@@d : {d-self.D}")
                rospy.loginfo(f"#GOAL : {goal_d}")
                if d > self.D + 0.2:
                    self.go_to_goal()
                else:
                    self.follow_wall()
                self.r.sleep()
            
            else:
                move_cmd = Twist()
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.0
                break


    def on_shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        plt.plot(list(range(len(self.errs))),
                    self.errs, label='errs')
        plt.axhline(y=0,color='R')
        plt.draw()
        plt.legend(loc="upper left", frameon=False)
        plt.savefig(f"errs_{self.k_p}_{self.k_d}_{self.k_i}.png")
        plt.show()

        rospy.sleep(1)
            

if __name__ == '__main__':
    try:
        pidc = PIDController()
        pidc.robot_action()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")