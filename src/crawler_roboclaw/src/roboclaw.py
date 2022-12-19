#!/usr/bin/env python
import roboclaw_driver.roboclaw_driver as roboclaw
import rospy
import tf
import numpy as np
from math import pi, cos, sin, atan2
from dynamic_reconfigure.server import Server
from roboclaw_node.cfg import RoboClawConfig
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion

from EncoderOdom import *
import roboclaw_driver.roboclaw_driver as roboclaw
import rospy
from geometry_msgs.msg import Quaternion, Twist, Vector3Stamped


__author__ = "bwbazemore@uga.edu (Brad Bazemore)"


class Node:
    def __init__(self):
        self.X = np.mat([[0.0],[0.0],[0.0]], dtype='float') #initial state for odomoetry
        self.accum_enc1 = 0 #accumulated encoder distances
        self.accum_enc2 = 0
        self.prev_enc1 = 0 #current raw encoder reading
        self.prev_enc2 = 0
        self.prev_enc =[None, None] 
        self.vel_l = 0.0
        self.vel_r = 0.0
        self.prev_theta = None
        self.prev_odom = None
        self.reset = False
        self.last_set_speed_time = None
        self.cmd_vel_cb_occured = False

        self.zero_count = 0
        self.stopped = False

        rospy.init_node("roboclaw_node")
        srv = Server(RoboClawConfig, self.config_callback)

        self.m2_pub = rospy.Publisher('m2_raw', Vector3Stamped, queue_size=1)
        self.m1_pub = rospy.Publisher('m1_raw', Vector3Stamped, queue_size=1)
        self.enc_pub = rospy.Publisher('m1m2_enc_accum', Vector3Stamped, queue_size=1)
        self.odom_pub = rospy.Publisher("~/odom_raw", Odometry, queue_size=1)
        self.broadcaster = tf.TransformBroadcaster()

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")
        
        #get params
        dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
        baud_rate = rospy.get_param("~baud", 115200)
        self.MAX_SPEED = float(rospy.get_param("~max_speed", "2.0"))
        self.ticks_per_meter = float(rospy.get_param("~ticks_per_meter", 651000))
        self.l = float(rospy.get_param("~base_width", "0.15")) #15 cm between the wheel 
        self.half_l = self.l/2.0 #half distance between wheels, this is just for conveneince
        self.angular_gain = float(rospy.get_param("~angular_gain",1.0))


        print("Angular Gain initial: ", self.angular_gain) #, type(self.angular_gain))
        print ("L initial: ", self.l) #, type(self.l))
        print ("ticks per meter initial: ", self.ticks_per_meter) #, type(self.ticks_per_meter))

        self.wheel_radius = rospy.get_param("~wheel_radius",0.07) #around 7cm
        self.odom_child_frame_id = rospy.get_param("~child_frame","/base_link") #around 7cm
        
        #get initial values
        self.m1kp =  rospy.get_param("~m1_pos_p")
        self.m1ki =  rospy.get_param("~m1_pos_i")
        self.m1kd =  rospy.get_param("~m1_pos_d")
        self.m1maxi = rospy.get_param("~m1_max_i")
        self.m2kp =  rospy.get_param("~m2_pos_p")
        self.m2ki =  rospy.get_param("~m2_pos_i")
        self.m2kd =  rospy.get_param("~m2_pos_d")
        self.m2maxi = rospy.get_param("~m2_max_i")
        self.deadzone = rospy.get_param("~deadzone")
        self.min_pos = rospy.get_param("~min_pos")
        self.max_pos = rospy.get_param("~max_pos")

        
        self.m1velp = .01
        self.m1veli = 0
        self.m1veld = 0
        self.m1velqpps = 200132
        self.m2velp = .01
        self.m2veli = 0
        self.m2veld = 0
        self.m2velqpps = 187055
        

        self.address = int(rospy.get_param("~address", "128"))
        if self.address > 0x87 or self.address < 0x80:
            rospy.logfatal("Address out of range")
            rospy.signal_shutdown("Address out of range")

        while True:
            try:
                roboclaw.Open(dev_name, baud_rate)
                roboclaw.SpeedM1M2(self.address, 0, 0)
                roboclaw.Close()
            except Exception as e:
                rospy.logfatal("Could not connect to Roboclaw")
                rospy.logdebug(e)
                #rospy.signal_shutdown("Could not connect to Roboclaw")
            
            rospy.sleep(1.)
            try:
                roboclaw.Open(dev_name, baud_rate)
            except Exception as e:
                rospy.logfatal("Could not connect to Roboclaw")
                rospy.logdebug(e)
                #rospy.signal_shutdown("Could not connect to Roboclaw")
            try:
                version = roboclaw.ReadVersion(self.address)
            except Exception as e:
                rospy.logwarn("Problem getting roboclaw version")
                rospy.logdebug(e)
            if not version[0]:
                rospy.logwarn("Could not get version from roboclaw")
                break
            else:
                rospy.logdebug(repr(version[1]))
                break
       
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        rospy.sleep(1)

    def publish_raw_encoder_data(self):
        
        m1_data = Vector3Stamped()
        m2_data = Vector3Stamped()
        current_time = rospy.Time.now()

        enc1 = roboclaw.ReadEncM1(self.address)
        enc2 = roboclaw.ReadEncM2(self.address)
        spdErr = roboclaw.GetSpeedError(self.address)
        posErr = roboclaw.GetPoseError(self.address)
        enc1spd = roboclaw.ReadSpeedM1(self.address)
        enc2spd = roboclaw.ReadSpeedM2(self.address)

        m1_data.header.stamp = current_time
        m1_data.header.frame_id = 'm1_raw_enc_data'
        m1_data.vector.x = enc1[1]
        m1_data.vector.y = spdErr[1]/float(self.ticks_per_meter)
        m1_data.vector.z = enc1spd[1]/float(self.ticks_per_meter)


        m2_data.header.stamp = current_time
        m2_data.header.frame_id = 'm1_raw_enc_data'
        m2_data.vector.x = enc2[1]
        m2_data.vector.y = spdErr[2]/float(self.ticks_per_meter)
        m2_data.vector.z = enc2spd[1]/float(self.ticks_per_meter)

        if enc1[0] and enc2[0] and spdErr[0] and posErr[0]:
            self.m1_pub.publish(m1_data)
            self.m2_pub.publish(m2_data)
            
            
    def accumulate_odom(self):
        enc1_cur = roboclaw.ReadEncM1(self.address)
        enc2_cur = roboclaw.ReadEncM2(self.address)

        # verify that there is a current reading, element 0 is just a check from the read
        # Then if the current reading is greater than the previous reading add the difference between
        # The current reading and the prior and save the reading as previous. 
        # If the current is less than the previous (meaning it was reset before this call), just add the 
        # current reading to the accumulated. 
        if (enc1_cur[0] and enc2_cur[0]):
            #check of odom has been reset to cause the difference between two readings
            if self.reset:
                self.accum_enc1 += enc1_cur[1]
                self.prev_enc1 = enc1_cur[1]
                self.accum_enc2 += enc2_cur[1]
                self.prev_enc2 = enc2_cur[1]
                self.reset = False
            else:
                self.accum_enc1 += enc1_cur[1] - self.prev_enc1
                self.prev_enc1 = enc1_cur[1]
                self.accum_enc2 += enc2_cur[1] - self.prev_enc2
                self.prev_enc2 = enc2_cur[1]

            #publish accumulated encoders
            enc_vals = Vector3Stamped()
            enc_vals.header.stamp = rospy.Time.now()
            enc_vals.header.frame_id = 'm1m2_enc_accum'
            enc_vals.vector.x = self.accum_enc1
            enc_vals.vector.y = self.accum_enc2
            self.enc_pub.publish(enc_vals)

            return [self.accum_enc1, self.accum_enc2]
        else:
            return [None,None] #if the read fails return zeros
    

    def publish_odom(self,enc):

        if self.prev_enc[0] == None:
            self.prev_enc = enc

        #calculate displacement in meters
        dr = (enc[0] - self.prev_enc[0])/self.ticks_per_meter 
        dl = (enc[1] - self.prev_enc[1])/self.ticks_per_meter
        self.prev_enc = enc

        #update state
        self.X[0,0] += ((dr+dl)/2)*cos(self.X[2,0])
        self.X[1,0] += ((dr+dl)/2)*sin(self.X[2,0])
        self.X[2,0] = atan2(sin(self.X[2,0]+(dl-dr)/self.half_l),cos(self.X[2,0]+(dl-dr)/self.half_l))

        #create odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = self.odom_child_frame_id
        
        odom.pose.pose.position.x = self.X[0,0] 
        odom.pose.pose.position.y = self.X[1,0]
        p = (odom.pose.pose.position.x,odom.pose.pose.position.y, 0)

        q = tf.transformations.quaternion_from_euler(0,0,self.X[2,0])
        odom.pose.pose.orientation = Quaternion(*q)
        
        self.odom_pub.publish(odom)
        #self.broadcaster.sendTransform(p, q, rospy.Time.now(), "base_link", "odom_raw")
    
    def cmd_vel_callback(self, twist):
        self.cmd_vel_cb_occured = True
        self.last_set_speed_time = rospy.get_rostime()
        linear_x = twist.linear.x
        angular_z = twist.angular.z
        deadzone = 0.01
        
        # check there are least 3 stopped commands before stopping
        # this avoids 0.0 commands when moving from forward to reverse velocity
        if (abs(twist.linear.x) <= deadzone) and (abs(twist.angular.z) <= deadzone):
            self.zero_count += 1
            if self.zero_count > 2:
                self.stopped = True
        else:
            self.zero_count = 0
            self.stopped = False

                     
        self.vel_l = (linear_x + self.angular_gain * self.half_l * angular_z)* self.ticks_per_meter
        self.vel_r = (linear_x - self.angular_gain * self.half_l * angular_z)* self.ticks_per_meter
    
    def config_callback(self,config, level):
        self.angular_gain = float(config['angular_gain'])
        self.l = float(config['base_width'])
        self.ticks_per_meter = float(config['ticks_per_meter'])
        self.m1velp =  config['m1_vel_p']
        self.m1veli =  config['m1_vel_i']
        self.m1veld =  config['m1_vel_d']
        self.m1velqpps = int(config['m1_vel_qpps'])
        self.m2velp =  config['m2_vel_p']
        self.m2veli =  config['m2_vel_i']
        self.m2veld =  config['m2_vel_d']
        self.m2velqpps = int(config['m2_vel_qpps'])
        self.m1kp = config['m1_pos_p']
        self.m1ki = config['m1_pos_i']
        self.m1kd = config['m1_pos_d']
        self.m1maxi = config['m1_max_i']
        self.m2kp = config['m2_pos_p']
        self.m2ki = config['m2_pos_i']
        self.m2kd = config['m2_pos_d']
        self.m2maxi = config['m2_max_i']
        self.deadzone = config['deadzone']

        return config
    
    def run(self):
        rospy.logdebug("Starting motor drive")
        already_stopped = False
        r_time = rospy.Rate(20)

        # while not rospy.is_shutdown():
        #     roboclaw.DutyM1M2(self.address,0x7FFF,0x7FFF)
        #     print "%s, %s"%(str(roboclaw.ReadSpeedM1(self.address)),str(roboclaw.ReadSpeedM2(self.address)))
            
        # roboclaw.DutyM1M2(self.address,0,0)
        while not rospy.is_shutdown():
            self.publish_raw_encoder_data()
            enc = self.accumulate_odom()
            if(enc[0] != None):
                self.publish_odom(enc)

            
            #NOTE: must check self.cmd_vel_cb_occurred is true first to prevent the other elements from checking
            # until after self.last_set_speed_time is set or the node will fail. 
            if (self.cmd_vel_cb_occured == False) or ((rospy.get_rostime() - self.last_set_speed_time).to_sec() > .1 or self.stopped):
                #rospy.loginfo("Did not get command for .2 second, stopping")
                #if failure on 
                while True:
                    try:
                        # TODO add when we set PID velocity this stops being able to fully get to 0
                        # need to maybe add a count to try a couple times? 
                        if already_stopped:
                            break
                        else:
                            print ("Locking")
                            already_stopped = True
                            #roboclaw.SetM1VelocityPID(self.address, 0.1, 0,0, self.m1velqpps)
                            #roboclaw.SetM2VelocityPID(self.address, 0.1, 0,0, self.m2velqpps)
                            roboclaw.SetM1PositionPID(self.address, self.m1kp, self.m1ki, self.m1kd, self.m1maxi, self.deadzone, self.min_pos, self.max_pos)
                            roboclaw.SetM2PositionPID(self.address, self.m2kp, self.m2ki, self.m2kd, self.m2maxi, self.deadzone, self.min_pos, self.max_pos)
                            #set encoders to 0, set acced decel rate, set goal to 0 (since encoders are reset)
                            roboclaw.ResetEncoders(self.address)
                            self.reset = True
                            accel = decel = 100000
                            speed = 10000
                            goal = 0
                            roboclaw.SpeedAccelDeccelPositionM1M2(self.address, accel, speed, decel, goal, accel, speed, decel, goal, 1)
                            break

                    except OSError as e:
                        rospy.logerr("Could not stop")
                        rospy.logdebug(e)
            else:
                try:
                    already_stopped = False
                    #configure pid (for reconfigure)
                    
                    #roboclaw.SetM1VelocityPID(self.address, self.m1velp, self.m1veli, self.m1veld, self.m1velqpps)
                    #roboclaw.SetM2VelocityPID(self.address, self.m2velp, self.m2veli, self.m2veld, self.m2velqpps)
                    roboclaw.SpeedM1M2(self.address, self.vel_r, self.vel_l)
                    
                except OSError as e:
                    rospy.logerr("Error sending motor commands")
                    rospy.logdebug(e)

            r_time.sleep()






    def shutdown(self):
        rospy.loginfo("Shutting down")
        try:
            roboclaw.ResetEncoders(self.address)
            accel = decel = 100000
            speed = 10000
            goal = 0
            roboclaw.SpeedAccelDeccelPositionM1M2(self.address, accel, speed, decel, goal, accel, speed, decel, goal, 1)
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                roboclaw.ResetEncoders(self.address)
                accel = decel = 100000
                speed = 10000
                goal = 0
                roboclaw.SpeedAccelDeccelPositionM1M2(self.address, accel, speed, decel, goal, accel, speed, decel, goal, 1)
            except OSError as e:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(e)


if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
