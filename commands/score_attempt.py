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
                if "L4" in self.elevator_and_arm.get_elevator_state():
                    arm_target = "score_left_L4"
                else:
                    arm_target = "score_left"
            else:
                if "L4" in self.elevator_and_arm.get_elevator_state():
                    arm_target = "score_right_L4"
                else:
                    arm_target = "score_right"
            self.elevator_and_arm.set_arm_state(arm_target)

        elif "score" in self.elevator_and_arm.get_arm_state():
            if "left" in self.elevator_and_arm.get_arm_state():
                arm_target = "stage_left"
            else:
                arm_target = "stage_right"
            self.elevator_and_arm.set_arm_state(arm_target)
        # else:
            # arm_target = self.elevator_and_arm.get_arm_state()

    def isFinished(self) -> bool:
        return True
