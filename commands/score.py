from commands2 import Command, SequentialCommandGroup
from wpilib import Timer

from subsystems.elevatorandarm import ElevatorAndArmSubsystem


class Score(Command):
    def __init__(self, elevator_and_arm: ElevatorAndArmSubsystem, timer: Timer):
        super().__init__()
        self.elevator_and_arm = elevator_and_arm
        self.timer = timer

        self.locked_in = False
        self.time_in = 10000000000000

        self.addRequirements(elevator_and_arm)

    def initialize(self):
        self.time_in = 10000000000000
        self.locked_in = False
        if "stage" in self.elevator_and_arm.get_arm_state():
            if "left" in self.elevator_and_arm.get_arm_state():
                arm_target = "score_left"
            else:
                arm_target = "score_right"
            self.elevator_and_arm.set_arm_state(arm_target)

    def execute(self):
        if self.elevator_and_arm.get_arm_at_target() and not self.locked_in:
            self.elevator_and_arm.intake.set(1)
            self.locked_in = True
            self.time_in = self.timer.get()

        if self.locked_in and self.timer.get() - self.time_in > 0.25:
            self.elevator_and_arm.set_arm_state("stow")

    def isFinished(self) -> bool:
        return self.timer.get() - self.time_in > 0.5

    def end(self, interrupted: bool):
        self.elevator_and_arm.intake.set(0)