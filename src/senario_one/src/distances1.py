#!/usr/bin/python3

from cmath import rect
from matplotlib import pyplot as plt
import numpy as np
import rospy
import tf
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

from math import atan2, pi, radians, sqrt
from senario_one.msg import co


class distances1:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        self.coPublisher = rospy.Publisher('ClosestObstacle', co, queue_size=10)
        self.book_shelf = (2.64, -1.55)
        self.dumpster = (1.23, -4.57)     
        self.barrel = (-2.51, -3.08)
        self.postbox = (-4.47, -0.57)
        self.brick_box = (-3.44, 2.75)
        self.cabinet = (-0.45, 4.05)
        self.cafe_table = (1.91, 3.37)
        self.fountain = (4.08, 1.14)
        self.obstacles = [("book_shelf", self.book_shelf), ("dumpster", self.dumpster), ("barrel", self.barrel), ("postbox", self.postbox), 
                            ("brick_box", self.brick_box), ("cabinet", self.cabinet), ("cafe_table", self.cafe_table), ("fountain", self.fountain)]

    def closestObstacle(nself, name, distance):
        closestObstacle = co()
        closestObstacle.obstacle_name = name
        closestObstacle.distance = distance
        return closestObstacle

    def run(self):
        while not rospy.is_shutdown():

            msg = rospy.wait_for_message("/odom" , Odometry) 
            curr_x = msg.pose.pose.position.x
            curr_y = msg.pose.pose.position.y
            distances = [(obstacle[0], sqrt((obstacle[1][0] - curr_x) ** 2 + (obstacle[1][1] - curr_y) ** 2)) for obstacle in self.obstacles]
            min_dis = min(distances, key= lambda x: x[1])
            co = self.closestObstacle(min_dis[0], min_dis[1])
            self.coPublisher.publish(co)




if __name__ == "__main__":
    controller = distances1()
    controller.run()