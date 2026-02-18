#!/usr/bin/env python

"""FlexBE state for navigating to a goal pose using Nav2."""

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus


class Drive2GoalState(EventState):
    """
    A FlexBE state that uses the Nav2 stack to navigate to a specified goal pose.

    This state sends a NavigateToPose action goal to the Nav2 stack and monitors
    the navigation progress until completion or failure.

    -- x            float   Target x position in meters
    -- y            float   Target y position in meters
    -- z            float   Target z position in meters
    -- qx           float   Quaternion x component for orientation
    -- qy           float   Quaternion y component for orientation
    -- qz           float   Quaternion z component for orientation
    -- qw           float   Quaternion w component for orientation
    -- frame_id     string  Reference frame for the goal pose (default: 'map')
    -- action_topic string  Nav2 action topic (default: 'navigate_to_pose')

    <= done         Navigation completed successfully, goal reached.
    <= failed       Navigation failed (action rejected, aborted, or canceled).
    """

    def __init__(self, x, y, z, qx, qy, qz, qw, frame_id='map', action_topic='navigate_to_pose'):
        super(Drive2GoalState, self).__init__(outcomes=['done', 'failed'])

        # Store goal pose parameters
        self._x = x
        self._y = y
        self._z = z
        self._qx = qx
        self._qy = qy
        self._qz = qz
        self._qw = qw
        self._frame_id = frame_id
        self._action_topic = action_topic

        # Initialize the action client
        ProxyActionClient.initialize(Drive2GoalState._node)
        self._client = ProxyActionClient({self._action_topic: NavigateToPose})

        # Track state
        self._error = False
        self._return_outcome = None

    def execute(self, userdata):
        """Check navigation status and return outcome when complete."""
        # If we already have an outcome (from previous execution while blocked), return it
        if self._return_outcome is not None:
            return self._return_outcome

        # Check if there was an error sending the goal
        if self._error:
            self._return_outcome = 'failed'
            return 'failed'

        # Check if the action has finished
        if self._client.has_result(self._action_topic):
            status = self._client.get_state(self._action_topic)

            if status == GoalStatus.STATUS_SUCCEEDED:
                Logger.loginfo('Navigation succeeded - goal reached!')
                self._return_outcome = 'done'
                return 'done'
            else:
                # Handle other terminal states (ABORTED, CANCELED, etc.)
                status_names = {
                    GoalStatus.STATUS_CANCELED: 'CANCELED',
                    GoalStatus.STATUS_ABORTED: 'ABORTED',
                    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
                }
                status_name = status_names.get(status, f'STATUS_{status}')
                Logger.logwarn(f'Navigation failed with status: {status_name}')
                self._return_outcome = 'failed'
                return 'failed'

        # Log feedback if available
        if self._client.has_feedback(self._action_topic):
            feedback = self._client.get_feedback(self._action_topic)
            if feedback is not None and hasattr(feedback, 'feedback'):
                fb = feedback.feedback
                if hasattr(fb, 'distance_remaining'):
                    Logger.loginfo(f'Distance remaining: {fb.distance_remaining:.2f}m')
                    # Custom success condition (user requested < 0.3m)
                    if fb.distance_remaining < 0.3:
                        Logger.loginfo('Goal reached based on custom tolerance (< 0.3m)!')
                        self._client.cancel(self._action_topic)
                        self._return_outcome = 'done'
                        return 'done'
            self._client.remove_feedback(self._action_topic)

        # Navigation still in progress
        return None

    def on_enter(self, userdata):
        """Send the navigation goal when entering the state."""
        self._error = False
        self._return_outcome = None

        # Create the goal pose
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._frame_id
        # Use empty timestamp (0,0) to use latest available transform
        # This avoids sim_time vs wall clock mismatches
        goal.pose.header.stamp.sec = 0
        goal.pose.header.stamp.nanosec = 0

        # Set position
        goal.pose.pose.position.x = float(self._x)
        goal.pose.pose.position.y = float(self._y)
        goal.pose.pose.position.z = float(self._z)

        # Set orientation (quaternion)
        goal.pose.pose.orientation.x = float(self._qx)
        goal.pose.pose.orientation.y = float(self._qy)
        goal.pose.pose.orientation.z = float(self._qz)
        goal.pose.pose.orientation.w = float(self._qw)

        Logger.loginfo(f'Navigating to position ({self._x}, {self._y}, {self._z}) '
                      f'with orientation ({self._qx}, {self._qy}, {self._qz}, {self._qw}) '
                      f'in frame "{self._frame_id}"')

        # Send the goal
        try:
            self._client.send_goal(self._action_topic, goal)
        except Exception as e:
            Logger.logerr(f'Failed to send NavigateToPose goal:\n{str(e)}')
            self._error = True

    def on_exit(self, userdata):
        """Cancel navigation if still active when exiting the state."""
        if self._client.is_active(self._action_topic):
            self._client.cancel(self._action_topic)
            Logger.loginfo('Cancelled active navigation goal.')

    def on_stop(self):
        """Cancel navigation when the behavior is stopped."""
        if self._client.is_active(self._action_topic):
            self._client.cancel(self._action_topic)
            Logger.loginfo('Cancelled navigation goal due to behavior stop.')
