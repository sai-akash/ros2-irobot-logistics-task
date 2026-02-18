#!/usr/bin/env python

"""FlexBE state for detecting objects using YOLO detections."""

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

from yolo_msgs.msg import DetectionArray


class FindObjectState(EventState):
    """
    A FlexBE state that checks if a specified object type is detected in YOLO detections.

    This state subscribes to the YOLO detection topic and checks if the specified
    object class is present in the current detections.

    Uses yolo_msgs from: https://git.faps.uni-erlangen.de/heengelhardt/yolo_msgs

    -- object_class     string  The class name of the object to search for (e.g., 'person', 'bottle')
    -- detection_topic  string  Topic for YOLO detections (default: '/yolo/detections')
    -- timeout          float   Maximum time to wait for detections in seconds (default: 1.0)

    <= available    The specified object was found in the detections.
    <= empty        The specified object was not found in the detections.
    """

    def __init__(self, object_class, detection_topic='/yolo/detections', timeout=1.0):
        super(FindObjectState, self).__init__(outcomes=['available', 'empty'])

        # Store parameters
        self._object_class = object_class.lower()
        self._detection_topic = detection_topic
        self._timeout = timeout

        # Initialize subscriber
        self._sub = ProxySubscriberCached({self._detection_topic: DetectionArray})

        # Track state
        self._start_time = None
        self._return_outcome = None

    def execute(self, userdata):
        """Check for the specified object in YOLO detections."""
        # If we already determined an outcome, return it
        if self._return_outcome is not None:
            return self._return_outcome

        # Check if we have detection messages
        if self._sub.has_msg(self._detection_topic):
            detections_msg = self._sub.get_last_msg(self._detection_topic)

            # Search through all detections for the target object
            if detections_msg.detections:
                for detection in detections_msg.detections:
                    # yolo_msgs/Detection has class_name field
                    if detection.class_name.lower() == self._object_class:
                        Logger.loginfo(f'Found object: "{detection.class_name}" '
                                      f'(class_id: {detection.class_id}, score: {detection.score:.2f})')
                        self._return_outcome = 'available'
                        return 'available'

            # Clear the message to check for new detections on next cycle
            self._sub.remove_last_msg(self._detection_topic)

        # Check timeout
        elapsed = self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds
        elapsed_sec = elapsed / 1e9

        if elapsed_sec > self._timeout:
            Logger.loginfo(f'Object "{self._object_class}" not found after {self._timeout}s timeout')
            self._return_outcome = 'empty'
            return 'empty'

        # Still searching
        return None

    def on_enter(self, userdata):
        """Reset state when entering."""
        self._start_time = self._node.get_clock().now()
        self._return_outcome = None

        # Clear any old cached messages
        if self._sub.has_msg(self._detection_topic):
            self._sub.remove_last_msg(self._detection_topic)

        Logger.loginfo(f'Searching for object: "{self._object_class}" on topic {self._detection_topic}')

    def on_exit(self, userdata):
        """Clean up when exiting the state."""
        pass
