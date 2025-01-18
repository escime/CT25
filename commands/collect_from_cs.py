from commands2 import Command

from subsystems.elevatorandarm import ElevatorAndArmSubsystem


class Collect(Command):
    def __init__(self, elevator_and_arm: ElevatorAndArmSubsystem):
        super().__init__()
        self.elevator_and_arm = elevator_and_arm

        self.addRequirements(elevator_and_arm)

    def initialize(self):
        self.elevator_and_arm.intake.set(-1)

    def isFinished(self) -> bool:
        return self.elevator_and_arm.intake_sensor_left.get() and self.elevator_and_arm.intake_sensor_right.get()

    def end(self, interrupted: bool):
        self.elevator_and_arm.intake.set(0)
