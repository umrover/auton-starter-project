import numpy as np

from context import Context
from drive import get_drive_command
from state_machine.state import State
from tag_seek import TagSeekState


class DriveState(State):
    def on_enter(self, context) -> None:
        pass

    def on_exit(self, context) -> None:
        pass

    def on_loop(self, context) -> State:
        target = np.array([7.0, -2.0, 0.0])

        # TODO: Get the rover's current pose. If we don't have a valid pose  from the system yet, stay in this DriveState and try again next loop.
        SE3_pose = context.rover.get_pose()
        if SE3_pose is None:
            return self

        # TODO: Use get_drive_command() to evaluate our route. You need to extract two things from this function: the drive command itself, and our completion status.
        # (HINT: Use thresholds 0.7 and 0.2 for completion_thresh and turn_in_place_thresh respectively.)
        drive_command, completion_status = get_drive_command(
            target_pos=target, rover_pose=SE3_pose, completion_thresh=0.7, turn_in_place_thresh=0.2)

        # TODO: If we are finished getting to the target, transition to the TagSeekState.
        if completion_status:
            return TagSeekState()

        # TODO: Send the drive command to the rover
        context.rover.send_drive_command(drive_command)

        # TODO: Tell state machine to stay in the DriveState (keep driving) by returning self
        return self