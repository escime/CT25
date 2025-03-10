from commands2 import Command, SequentialCommandGroup
from wpilib import Timer
from subsystems.elevatorandarm import ElevatorAndArmSubsystem


class AutoScore(Command):
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
        if "stage" or "score" in self.elevator_and_arm.get_arm_state() and "manual" not in self.elevator_and_arm.get_elevator_state():
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
            if "L4" in self.elevator_and_arm.get_elevator_state():
                self.elevator_and_arm.set_elevator_state("L4_scoring")
                self.elevator_and_arm.intake.set(0.35)
            elif "L3" in self.elevator_and_arm.get_elevator_state():
                self.elevator_and_arm.set_elevator_state("L3_scoring")
                self.elevator_and_arm.intake.set(0.35)
            elif "L2" in self.elevator_and_arm.get_elevator_state():
                self.elevator_and_arm.set_elevator_state("L2_scoring")
                self.elevator_and_arm.intake.set(0.35)

    def execute(self):
        if self.elevator_and_arm.get_arm_at_target() and self.elevator_and_arm.get_elevator_at_target() and not self.locked_in:
            self.elevator_and_arm.intake.set(0.35)
            self.locked_in = True
            self.time_in = self.timer.get()

        if self.locked_in and self.timer.get() - self.time_in > 0.05:
            self.elevator_and_arm.set_arm_state("stow")

    def isFinished(self) -> bool:
        # print(self.elevator_and_arm.get_arm_at_target())
        return self.timer.get() - self.time_in > 0.1

    def end(self, interrupted: bool):
        self.elevator_and_arm.intake.set(0)
