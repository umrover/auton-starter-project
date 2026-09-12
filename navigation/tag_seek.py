from geometry_msgs.msg import Twist

from state_machine.state import State
from state import DoneState, FailState



# The simulator publishes 640-pixel-wide images on /zed/left/image. The new
# perception solution publishes the tag center as an absolute pixel coordinate.
CAMERA_WIDTH_PIXELS = 640.0
CAMERA_CENTER_X_PIXELS = CAMERA_WIDTH_PIXELS / 2.0

# Normalized error uses:
#     (x_pixel - 320.0) / 640.0
# 
# 0.05 is 5% of the image width, or 32 pixels. This is a reasonable tolerance for centering the tag in the image before driving forward. 
# The rover will drive forward while the tag is within this tolerance, and will turn in place if the tag is outside of this tolerance.
CENTERING_TOLERANCE = 0.05

# The old closeness metric was 1 - area_ratio and used a threshold of 0.995,
# equivalent to a tag occupying 0.5% of the image. The new perception metric is
# 1 - sqrt(area_ratio), so the equivalent threshold is about 0.929.
CLOSENESS_THRESHOLD = 0.93

SEARCH_ANGULAR_SPEED = 0.75
TURN_ANGULAR_SPEED = 0.25
DRIVE_SPEED = 1.0

# Navigation runs at 60 Hz. Allow enough time for a complete search rotation at
# SEARCH_ANGULAR_SPEED before declaring failure.
TAG_FAILURE_TOLERANCE = 60 * 30



class TagSeekState(State):
    cur_failed_detections: int = 0

    def on_enter(self, context) -> None:
        self.cur_failed_detections = 0

    def on_exit(self, context) -> None:
        pass

    def on_loop(self, context) -> State:
        # TODO: get the tag's location and properties (HINT: use get_fid_data() from context.env)

        # TODO: if we don't have a tag (None or -1): go to the FailState after TAG_FAILURE_TOLERANCE iterations (HINT: use cur_failed_detections to keep track of the amount of failures)

        # Find how far the tag is from the camera center, normalized by the total width 
        # (negative = left). This allows us to check if we are centered on the target 
        # and within the required closeness threshold.
        horizontal_error = (
            tag.x_tag_center_pixel - CAMERA_CENTER_X_PIXELS
        ) / CAMERA_WIDTH_PIXELS
        is_centered = abs(horizontal_error) < CENTERING_TOLERANCE
        is_close_enough = tag.closeness_metric < CLOSENESS_THRESHOLD

        # TODO: if we are within angular and distance tolerances: go to DoneState (HINT: use tag.x_tag_center_pixel and tag.closeness_metric)

        # TODO: figure out the Twist command to be applied to move the rover closer to the tag (HINT: Think about how the heading of the rover should be orientated before driving to the tag)        

        # TODO: send Twist command to rover

        # TODO: stay in the TagSeekState (with outcome "working")
