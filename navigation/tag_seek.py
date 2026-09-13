from geometry_msgs.msg import Twist

from state_machine.state import State
from state import DoneState, FailState


# The simulator publishes 640-pixel-wide images on /zed/left/image. The new
# perception solution publishes the tag center as an absolute pixel coordinate.
CAMERA_WIDTH_PIXELS = 640.0
CAMERA_CENTER_X_PIXELS = CAMERA_WIDTH_PIXELS / 2.0

# 0.05 is 5% of the image width, or 32 pixels. This is a reasonable tolerance for centering the tag in the image before driving forward. 
# The rover will drive forward while the tag is within this tolerance, and will turn in place if the tag is outside of this tolerance.
CENTERING_TOLERANCE = 0.05

# You defined how closeness_metric is calculated in Perception. So you know it must produce a value from 0.0 to 1.0, where smaller values mean the tag is closer.
# Calibrate your threshold value by observing the topic /tag when the rover is at the desired stopping distance.
CLOSENESS_THRESHOLD = 0.93  # TODO: Tune for your perception implementation.

# --- Movement Speeds ---
# SEARCH_ANGULAR_SPEED: How fast the rover spins in place when the tag is completely lost. It is relatively fast to scan the area quickly.
SEARCH_ANGULAR_SPEED = 0.75

# TURN_ANGULAR_SPEED: How fast the rover turns to center the tag once it is in view. 
# This is deliberately slower than the search speed to prevent overshooting the target.
TURN_ANGULAR_SPEED = 0.25

# DRIVE_SPEED: The forward linear speed applied to close the distance to the tag.
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
        # TODO: Fetch the tag's location and properties 
        # (HINT: use get_fid_data() from context.env)
        tag = context.env.get_fid_data()

        # TODO: Implement lost-tag behaviour. If a valid tag is not found (None or -1), command the rover to spin in place to search for it. 
        # (HINT: Track the number of consecutive failed detections, if it exceeds TAG_FAILURE_TOLERANCE, halt the rover and transition to the FailState.)
        if tag is None or tag.tag_id == -1:
            self.cur_failed_detections += 1

            if self.cur_failed_detections >= TAG_FAILURE_TOLERANCE:
                context.rover.send_drive_stop()
                return FailState()

            search_command = Twist()
            search_command.angular.z = SEARCH_ANGULAR_SPEED
            context.rover.send_drive_command(search_command)
            return self

        self.cur_failed_detections = 0

        # TODO: Calculate the tag's horizontal error as a fraction of the total camera width.
        # (HINT: Use the tag's x-coordinate and the CAMERA_* constants defined above. Expected output: 0.0 is dead center, < 0.0 is left, and > 0.0 is right.)
        horizontal_error = (
            tag.x_tag_center_pixel - CAMERA_CENTER_X_PIXELS
        ) / CAMERA_WIDTH_PIXELS

        # TODO: Create two boolean variables evaluating the rover's position:
        # 1. Is the tag centered within our acceptable tolerance?
        # 2. Is the rover close enough to the target?
        is_centered = abs(horizontal_error) < CENTERING_TOLERANCE
        is_close_enough = tag.closeness_metric < CLOSENESS_THRESHOLD

        # TODO: Handle the success condition. If the tag is both centered and close enough, halt the rover and transition to the DoneState.
        if is_close_enough and is_centered:
            context.rover.send_drive_stop()
            return DoneState()

        # TODO: Construct a Twist command to define the rover's drive-to-tag behavior. 
        # (HINT: Think about the physical movement required: how should the distance and centering booleans influence forward speed versus rotational speed? 
        # If the rover needs to turn, how do you determine the direction? Use TURN_ANGULAR_SPEED and DRIVE_SPEED)
        twist = Twist()

        if not is_close_enough:
            twist.linear.x = DRIVE_SPEED

        if not is_centered:
            if horizontal_error < 0:
                # Tag is to the left of the image center, so turn left
                twist.angular.z = TURN_ANGULAR_SPEED 
            else:
                # Tag is to the right of the image center, so turn right
                twist.angular.z = -TURN_ANGULAR_SPEED         

        # TODO: Send Twist command to rover
        context.rover.send_drive_command(twist)

        # TODO: Remain in the current state (TagSeekState)
        return self