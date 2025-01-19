from commands2 import Command

from subsystems.elevatorandarm import ElevatorAndArmSubsystem


class ScoreAttempt(Command):
    def __init__(self, elevator_and_arm: ElevatorAndArmSubsystem):
        super().__init__()
        self.elevator_and_arm = elevator_and_arm

        self.addRequirements(elevator_and_arm)

    def initialize(self):
        if "stage" in self.elevator_and_arm.get_arm_state():
            if "left" in self.elevator_and_arm.get_arm_state():
                arm_target = "score_left"
            else:
                arm_target = "score_right"
        elif "score" in self.elevator_and_arm.get_arm_state():
            if "left" in self.elevator_and_arm.get_arm_state():
                arm_target = "stage_left"
            else:
                arm_target = "stage_right"
        else:
            arm_target = self.elevator_and_arm.get_arm_state()
        self.elevator_and_arm.set_arm_state(arm_target)

    def isFinished(self) -> bool:
        return True
