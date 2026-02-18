#!/usr/bin/env python

import rclpy
from rclpy.duration import Duration
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class StopState(EventState):
    '''
    This state publishes a constant zero TwistStamped command based on parameters.  The state monitors the
    robot odometry message and returns a failed outcome if speed is not near zero within the timeout

    -- timeout         float     Time which needs to have passed since the behavior started. (default: 0.25)
    -- odom_topic      string    topic of the robot odometry message (default: 'odom')
    -- cmd_topic       string    topic name of the robot velocity command (default: 'cmd_vel')
    <= done         Robot stopped within the specified time.
    <= failed       The robot is still moving according to the odometry message after timeout.
    '''

    def __init__(self, timeout=0.25, cmd_topic='cmd_vel', odom_topic='odom'):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(StopState, self).__init__(outcomes = ['done', 'failed'])

        ProxyPublisher.initialize(StopState._node)
        ProxySubscriberCached.initialize(StopState._node)

        # Store state parameter for later use.
        self._timeout           = Duration(seconds=timeout)
        self._twist             = Twist()#TwistStamped()


        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._start_time = None

        self._done       = None # Track the outcome so we can detect if transition is blocked

        self._odom_topic   = odom_topic
        self._cmd_topic    = cmd_topic
        self._odom_sub     = ProxySubscriberCached({self._odom_topic: Odometry})
        self._pub          = ProxyPublisher({self._cmd_topic: Twist})

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # If no outcome is returned, the state will stay active.
        if (self._done):
            # We have completed the state, and therefore must be blocked by autonomy level
            # Stop the robot, but and return the prior outcome
            ts = Twist()#TwistStamped() # Zero twist to stop if blocked
            #ts.header.stamp = self._node.get_clock().now().to_msg()
            self._pub.publish(self._cmd_topic, ts)
            return self._done


        if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._timeout.nanoseconds:
            # Normal completion, verify stoppage
            if (self._sub.has_msg(self._odom_topic)):
                odom = self._sub.get_last_msg(self._odom_topic)
                speed = odom.twist.twist.linear.x*odom.twist.twist.linear.x + odom.twist.twist.angular.z*odom.twist.twist.angular.z
                if (speed > 5.0e-4):
                    Logger.logwarn('Timed Stop failed - current twist: linear = %f,%f,%f angular=%f, %f, %f' %
                        (odom.twist.twist.linear.x,  odom.twist.twist.linear.y,  odom.twist.twist.linear.z,
                         odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z))
                    self._done = 'failed'
                    return 'failed'
                else:
                    self._done = 'done'
                    return 'done'
            else:
                Logger.logwarn('Timed Stop failed - no odometry feedback!')
                self._done = 'failed'
                return 'failed'


        # Normal operation publish the zero twist
        #self._twist.header.stamp = self._node.get_clock().now().to_msg()  # update the time stamp
        self._pub.publish(self._cmd_topic, self._twist)
        return None

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        #self._twist.header.stamp = self._node.get_clock().now().to_msg()  # update the time stamp
        self._pub.publish(self._cmd_topic, self._twist)

        self._start_time = self._node.get_clock().now()
        self._done       = None # reset the completion flag
