from commands2 import Command

from subsystems.elevatorandarm import ElevatorAndArmSubsystem


class Collect(Command):
    def __init__(self, elevator_and_arm: ElevatorAndArmSubsystem):
        super().__init__()
        self.elevator_and_arm = elevator_and_arm

        self.collection_buffer = [False] * 100

        self.addRequirements(elevator_and_arm)

    def initialize(self):
        self.elevator_and_arm.intake.set(-1)
        self.collection_buffer = [False] * 40

    def execute(self):
        if not self.elevator_and_arm.intake_sensor_left.get() and not self.elevator_and_arm.intake_sensor_right.get():
            self.collection_buffer[0] = True
        self.collection_buffer = self.collection_buffer[1:] + self.collection_buffer[:1]

    def isFinished(self) -> bool:
        return all(self.collection_buffer)

    # def end(self, interrupted: bool):
    #     self.elevator_and_arm.intake.set(0)
