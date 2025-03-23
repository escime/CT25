from commands2 import Command

from subsystems.elevatorandarm import ElevatorAndArmSubsystem


class SwapArm(Command):
    def __init__(self, elevator_and_arm: ElevatorAndArmSubsystem):
        super().__init__()
        self.elevator_and_arm = elevator_and_arm

        self.addRequirements(elevator_and_arm)

    def initialize(self):
        if "left" in self.elevator_and_arm.get_arm_state():
            if "L4" in self.elevator_and_arm.get_arm_state():
                arm_target = self.elevator_and_arm.get_arm_state()[:-7] + "right_L4"
            else:
                arm_target = self.elevator_and_arm.get_arm_state()[:-4] + "right"
            self.elevator_and_arm.set_arm_state(arm_target)
        elif "right" in self.elevator_and_arm.get_arm_state():
            if "L4" in self.elevator_and_arm.get_arm_state():
                arm_target = self.elevator_and_arm.get_arm_state()[:-8] + "left_L4"
            else:
                arm_target = self.elevator_and_arm.get_arm_state()[:-5] + "left"
            self.elevator_and_arm.set_arm_state(arm_target)
        # elif "stow" in self.elevator_and_arm.get_arm_state():
        #     self.elevator_and_arm.set_arm_state("stage_right_L4")


    def isFinished(self) -> bool:
        return True
