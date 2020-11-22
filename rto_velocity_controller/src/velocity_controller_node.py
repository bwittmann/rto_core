#!/usr/bin/env python3

import rospy
import os
from threading import Lock

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class VelocityControllerNode():
    '''
    Node used for implementing a velocity controller that modifies twist messages obtained from the /input/cmd_vel 
    topic and publishes the controlled twist messages at the topic /pioneer/cmd_vel or /cmd_vel depending on the type
    of robot.
    '''

    def __init__(self):
        '''
        Initialize an instance of the VelocityControllerNode class, including the node and its publishers
        and subscribers.
        '''
        # Init member variables
        self.sim_lock = Lock()

        # Init publishers depending on the used robot by checking the environment variable ROBOT
        if os.environ['ROBOT'] == 'p3dx':
            self.pub_cmd = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=10)
        elif os.environ['ROBOT'] == 'rto-1':
            self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        else:
            rospy.loginfo("Environment variable ROBOT has to be set to 'p3dx' or 'rto-1'")
            
        # Init subscribers
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.cb_scan)
        self.sub_cmd_input = rospy.Subscriber('/input/cmd_vel', Twist, self.cb_cmd_input)

        # Init messages for publishing
        self.msg_cmd = Twist()

        # Init instant constants from parameter server
        self.MAX_VEL = rospy.get_param('~/max_velocity')
        self.MAX_VEL_FACTOR_BACKW = rospy.get_param('~/max_velocity_factor_backw')
        self.BREAK_START_DIST = rospy.get_param('~/break_start_dist')
        self.PASS_DISTANCE = rospy.get_param('~/pass_distance')
        self.STOP_DIST = rospy.get_param('~/stop_dist')

        # Init instant variables
        self.ranges_laser = [-1]*245
        self.cmd_input = [0,0,0,0,0,0]

    def cb_scan(self, msg):
        '''
        Callback function for retrieving the current LaserScan messages.
        @param msg: Message containing information of the scan like angle_min, angle_max, ranges or frame_id.
        '''
        self.sim_lock.acquire()
        self.ranges_laser = list(msg.ranges)
        self.sim_lock.release()

    def cb_cmd_input(self, msg):
        '''
        Callback funktion for retrieving the current Twist messages.
        @param msg: Message containing an angular and linear velocity for the x,y and z direction.
        '''
        self.sim_lock.acquire()
        self.cmd_input = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z] 
        self.sim_lock.release()

    def run(self, rate=40):
        '''
        Method that modifies the Twist messages from the input topic and publishes them on the output topic.
        @param rate: Determines the frequency in which the modified messages are published. A rate of 40 corresponds
        to 40 Hz, which is exactly the frequency the laser scaner is capable of. 
        '''
        while not rospy.is_shutdown():

            # Minumum range to an obstacle 
            min_range = min(self.ranges_laser)
            # Minumum range to an obstacle detected in the front of the robot
            min_range_front = min(self.ranges_laser[88:158])
            # Minumum range to an obstacle detected to the right of the robot
            min_range_right = min(self.ranges_laser[0:88])
            # Minumum range to an obstacle detected to the left of the robot
            min_range_left = min(self.ranges_laser[158:245])


            # Adjustment of linear velocity in the robots x direction based on a proportional velocity model
            if min_range < self.STOP_DIST and self.cmd_input[0] > 0:
                # Emergency stop for additional safety
                self.msg_cmd.linear.x = 0
            elif min_range_front < self.BREAK_START_DIST and self.cmd_input[0] > 0:
                # When directly driving towards an object the robot should start to break early to reduce wear
                self.msg_cmd.linear.x = max(0, (self.MAX_VEL/(self.BREAK_START_DIST - self.STOP_DIST) * min_range_front - self.MAX_VEL/(self.BREAK_START_DIST - self.STOP_DIST) * self.STOP_DIST))
            elif min_range < self.PASS_DISTANCE and self.cmd_input[0] > 0:
                # When passing an object the regulations regarding the distance to keep should be not as strict
                self.msg_cmd.linear.x = max(0, (self.MAX_VEL/(self.PASS_DISTANCE - self.STOP_DIST) * min_range - self.MAX_VEL/(self.PASS_DISTANCE - self.STOP_DIST) * self.STOP_DIST))
            elif self.cmd_input[0] < 0:
                # Since no 360 degree laserscan is attached, driving back only very slow to not hit an obstacle
                self.msg_cmd.linear.x = self.cmd_input[0] * self.MAX_VEL_FACTOR_BACKW
            else:
                self.msg_cmd.linear.x = self.cmd_input[0]


            # Adjustment of linear velocity in the robots y direction based on a proportional velocity model
            if min_range_left < self.BREAK_START_DIST and self.cmd_input[1] > 0:
                # When directly driving towards an object the robot should start to break early to reduce wear
                self.msg_cmd.linear.y = max(0, (self.MAX_VEL/(self.BREAK_START_DIST - self.STOP_DIST) * min_range_left - self.MAX_VEL/(self.BREAK_START_DIST - self.STOP_DIST) * self.STOP_DIST))
            elif min_range_right < self.BREAK_START_DIST and self.cmd_input[1] < 0:
                # When directly driving towards an object the robot should start to break early to reduce wear
                self.msg_cmd.linear.y = - max(0, (self.MAX_VEL/(self.BREAK_START_DIST - self.STOP_DIST) * min_range_right - self.MAX_VEL/(self.BREAK_START_DIST - self.STOP_DIST) * self.STOP_DIST))
            else:
                self.msg_cmd.linear.y = self.cmd_input[1]
           

            self.msg_cmd.linear.z = self.cmd_input[2]
            self.msg_cmd.angular.x = self.cmd_input[3]
            self.msg_cmd.angular.y = self.cmd_input[4]
            self.msg_cmd.angular.z = self.cmd_input[5]


            self.pub_cmd.publish(self.msg_cmd)

        
            rospy.loginfo("Min. distance recorded by laser: {}".format(round(min_range, 2)))
            
            # Additional logging for debugging
            #rospy.loginfo("Min. dist. front: {}".format(round(min_range_front, 2)))
            #rospy.loginfo("Min. dist. left: {}".format(round(min_range_left, 2)))
            #rospy.loginfo("Min. dist. right: {}".format(round(min_range_right, 2)))
            #rospy.loginfo(round(self.msg_cmd.linear.x, 2))
            #rospy.loginfo(round(self.msg_cmd.linear.y, 2))

        
            rospy.sleep(1/rate)


        
if __name__ == "__main__":
    rospy.init_node('velocity_controller_node')

    velocity_controller_node = VelocityControllerNode()

    # Run the velocity controller
    velocity_controller_node.run()

