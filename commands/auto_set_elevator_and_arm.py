from commands2 import Command
from subsystems.elevatorandarm import ElevatorAndArmSubsystem


class AutoSetElevatorAndArm(Command):
    def __init__(self, setpoint: str, side: str, elevator_and_arm: ElevatorAndArmSubsystem):
        super().__init__()
        self.setpoint = setpoint
        self.elevator_and_arm = elevator_and_arm
        self.side = side

        self.wait_for = "none"
        self.arm_target = "none"

        self.addRequirements(elevator_and_arm)

    def initialize(self):
        if self.setpoint == "stow" or \
                self.setpoint == "L1" or \
                self.setpoint == "L2" or \
                self.setpoint == "L3" or \
                self.setpoint == "L4":

            if self.setpoint != "stow":
                self.arm_target = "stage_" + self.side
            else:
                self.arm_target = "stow"

            self.elevator_and_arm.set_elevator_state(self.setpoint)
            if self.elevator_and_arm.get_arm_state() != "stow":
                self.elevator_and_arm.set_arm_state("stow")

    def isFinished(self) -> bool:
        return self.elevator_and_arm.get_elevator_at_target()

    def end(self, interrupted: bool):
        if not interrupted:
            self.elevator_and_arm.set_arm_state(self.arm_target)
