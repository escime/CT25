from commands2 import Command
from subsystems.elevatorandarm import ElevatorAndArmSubsystem
from phoenix6.utils import get_current_time_seconds

class ThrowAlgae(Command):

    def __init__(self, elevator_and_arm: ElevatorAndArmSubsystem):
        super().__init__()

        self.elevator_and_arm = elevator_and_arm
        self.algae_thrown = False
        self.start_time = 10000000

        self.addRequirements(self.elevator_and_arm)

    def initialize(self):
        self.elevator_and_arm.set_elevator_state("net")
        # self.elevator_and_arm.intake.set(0)
        self.algae_thrown = False

    def execute(self):

        if self.elevator_and_arm.get_elevator_position() >= 5.5:
            self.elevator_and_arm.intake.set(0)

        if self.elevator_and_arm.get_elevator_position() >= 4 and not self.algae_thrown:
            self.elevator_and_arm.set_arm_state("stage_right")
            # self.elevator_and_arm.intake.set(0)
            self.algae_thrown = True
            self.start_time = get_current_time_seconds()

        # if self.elevator_and_arm.get_arm_position() <= 0.1 and self.algae_thrown:
        #     self.elevator_and_arm.intake.set(0)

    def isFinished(self) -> bool:
        return self.algae_thrown and get_current_time_seconds() - self.start_time > 1
