from typing import Tuple

import numpy as np
from geometry_msgs.msg import Twist
from util.SE3 import SE3

MAX_DRIVING_SPEED = 1.0
MAX_ANGULAR_SPEED = 1.5
TURNING_P = 1.0

def get_drive_command(
    target_pos: np.ndarray,
    rover_pose: SE3,
    completion_thresh: float,
    turn_in_place_thresh: float,
) -> Tuple[Twist, bool]:
    """
    Calculate the velocity command that drives the rover towards a target position, and whether we are within the completion threshold.

    :param target_pos:              Target position to drive to.
    :param rover_pose:              Current rover pose.
    :param completion_thresh:       If the distance to the target is less than this stop.
    :param turn_in_place_thresh     Maximum heading error in radians at which the rover may drive forward.
                                    Above this angle, the rover will turn in place to align with the target.
                                    
    :return: Velocity command and whether the rover reached the target
    """
    if completion_thresh <= 0.0:
        raise ValueError(f"Argument {completion_thresh} should be greater than 0")

    if not (0.0 < turn_in_place_thresh < np.pi):
        raise ValueError(f"Argument {turn_in_place_thresh} should be between 0 and π radians")
    
    # Navigation is planar, so only use x and y coordinates. 

    rover_pos = rover_pose.position[:2]
    target_pos = target_pos[:2]
    target_delta = target_pos - rover_pos
    target_dist = np.linalg.norm(target_delta)

    # Check completion threshold before normalizing the target direction, to avoid dividing by zero

    if target_dist <= completion_thresh:
        return Twist(), True

    # Normalize direction (turn vector into a unit vector)

    target_dir = target_delta / target_dist

    # Obtain and normalize the rover's planar forward direction.

    rover_dir = rover_pose.rotation.direction_vector()[:2]
    rover_dir_norm = np.linalg.norm(rover_dir)

    if rover_dir_norm <= np.finfo(float).eps:
        raise ValueError("The rover heading has no planar component")

    rover_dir = rover_dir / rover_dir_norm

    # Imagine two arrows viewed from above:
    #      rover_dir  points where the front of the rover is facing
    #      target_dir points from the rover to the target
    #
    # The dot product of these two vectors tell us how aligned they are.
    #   1  means directly ahead.
    #   0  means 90 degrees to either side.
    #   -1 means directly behind.
    # 
    # np.clip just forces the dot product to be between -1 and 1, which is the valid range for the arccos function. 
    # This is a safety measure to avoid numerical issues (floating point rounding errors).

    dot_alignment = np.clip(np.dot(target_dir, rover_dir), -1.0, 1.0)

    # The z-component of the cross product of tells us which way to turn to align with the target.
    #   positive means the target is to the left of the rover.
    #   negative means the target is to the right of the rover.
    #   zero means the target is directly ahead or behind.
    #
    # Although both arrows lie in the x-y plane, their cross product points perpendicular to that plane,
    # along the z-axis. We only care about the sign of that z-component, so we don't need to compute the full cross product.

    cross_z = (
        rover_dir[0] * target_dir[1]
        - rover_dir[1]  * target_dir[0]
    )

    # For unit vectors, dot = cos(theta) and cross_z = sin(theta). Combining
    # them with atan2 gives us the signed angle from rover_dir to target_dir, which is the heading error we want to correct.
    # It will also give us the correct sign for the angular velocity command, so we turn in the right direction.
    heading_error = np.arctan2(cross_z, dot_alignment)

    cmd_vel = Twist()

    # Remaining heading error is used to scale angular speed, so a larger heading error results in a faster turn.
    # The TURNING_P constant is a proportional gain that can be tuned to adjust the responsiveness of the rover's turning behavior.
    # While the turning command approaches zero as the rover aligns with the target, the sign of the command is preserved, 
    # so the rover will turn in the correct direction. Positive angular.z means turn left, negative means turn right.
    # The clip keeps the angular speed within the maximum allowed range, preventing excessive turning speeds that could destabilize the rover.
    cmd_vel.angular.z = float(
        np.clip(heading_error * TURNING_P, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
    )

    # Turn in place for large heading errors. Once sufficiently aligned,
    # drive forward while making small heading corrections. 
    # The rover will stop when it is within the completion threshold of the target.

    if abs(heading_error) <= turn_in_place_thresh:
        # Remaining distance is used to scale linear speed, so the rover slows down as it approaches the target.
        cmd_vel.linear.x = float(
            np.clip(target_dist, 0.0, MAX_DRIVING_SPEED,)
        )        

    return cmd_vel, False