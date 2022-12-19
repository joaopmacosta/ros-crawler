#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class CrawlerTeleop :
    
    def __init__(self):
        self.axis_linear = None
        self.axis_angular = None
        self.scale_linear = None
        self.scale_angular = None
        self.pub = None
        self.sub = None

    def joy_callback(self, msg):
        twist = Twist()
        twist.angular.z = self.scale_angular * msg.axes[self.axis_angular]
        twist.linear.x = self.scale_linear * msg.axes[self.axis_linear]
        rospy.loginfo("Speed: %.3f | Angle: %.3f", twist.linear.x, twist.angular.z)
        
        self.pub.publish(twist)
    
    def run(self):
        rospy.init_node("crawler_teleop")
        rospy.logwarn("crawler_teleop node started!")
        
        self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        sub = rospy.Subscriber("/joy",Joy, self.joy_callback, queue_size=1)
        

        self.axis_linear = rospy.get_param("~axis_linear",1)
        self.axis_angular = rospy.get_param("~axis_angular",0)
        self.scale_linear = rospy.get_param("~scale_linear",1.0)
        self.scale_angular = rospy.get_param("~scale_angular",1.0)

        rospy.spin()

if __name__ == '__main__':
    teleop = CrawlerTeleop()
    teleop.run()


