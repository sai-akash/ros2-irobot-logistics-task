#!/usr/bin/env python

import math
import rclpy
from rclpy.duration import Duration
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


class DriveDistanceState(EventState):
    """
    This state lets a mobile robot drive a certain distance by publising a Twist command based on parameters and checking odometry.

    -- target_time     float     Time which needs to have passed since the behavior started.
    -- velocity        float     Body velocity (m/s)
    -- distance        float     Driving distance (m)
    -- cmd_topic       string    topic name of the robot velocity command (default: 'cmd_vel')
    -- odom_topic      string    topic name of the robot odometry (default: 'odom')
    <= done            System is driven the given distance.
    """

    def __init__(self, target_time, velocity, distance, cmd_topic='cmd_vel', odom_topic='odom'):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(DriveDistanceState, self).__init__(outcomes = ['done'])

        ProxyPublisher.initialize(DriveDistanceState._node)

        # Store state parameter for later use.
        self._target_time           = Duration(seconds=target_time)
        self._twist                 = Twist()#TwistStamped()
        self._twist.linear.x        = velocity    #twist.linear.x  = velocity

        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._start_time = None

        self._done       = None # Track the outcome so we can detect if transition is blocked

        self._distance     = distance
        self._odom_topic   = odom_topic
        self._cmd_topic    = cmd_topic
        self._odom_sub     = ProxySubscriberCached({self._odom_topic: Odometry})
        self._pub          = ProxyPublisher({self._cmd_topic: Twist})
        self._odom         = Odometry()
        self._init_position= Point()
        
    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # If no outcome is returned, the state will stay active.
        
        #Get odom topic
        if (self._sub.has_msg(self._odom_topic)):
            self._odom = self._sub.get_last_msg(self._odom_topic)
            
        #Calculate distance between current and initial pose
        cur_distance = math.sqrt(math.pow(self._odom.pose.pose.position.x-self._init_position._x, 2)+ math.pow(self._odom.pose.pose.position.y-self._init_position._y, 2))
        Logger.loginfo('Current distance: ' + str(cur_distance))
        
        #Publish cmd_vel topic
        if(cur_distance<self._distance):
            self._pub.publish(self._cmd_topic, self._twist)
            return None
        else:
            ts = Twist()
            self._pub.publish(self._cmd_topic, ts)
            self._done = 'done'
            return 'done'

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        self._start_time = self._node.get_clock().now()
        if (self._sub.has_msg(self._odom_topic)):
            init_odom = self._sub.get_last_msg(self._odom_topic)
            #Save inital pose
            self._init_position = init_odom.pose.pose.position
            Logger.loginfo('Start at position: ' + str(self._init_position))

        self._done       = None # reset the completion flag
