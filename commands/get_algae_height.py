from commands2 import Command
from wpilib import DriverStation
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.utilsubsystem import UtilSubsystem
from math import sqrt


class GetAlgaeHeight(Command):
    def __init__(self, drive: CommandSwerveDrivetrain, util: UtilSubsystem):
        super().__init__()
        self.drive = drive
        self.util = util

        self.algae_height = "algae_low"

    def initialize(self):
        self.algae_height = self.get_algae_height()

    def isFinished(self) -> bool:
        # print(self.elevator_and_arm.get_arm_at_target())
        return True

    def end(self, interrupted: bool):
        self.util.algae_height = self.algae_height

    def get_algae_height(self) -> str:
        pose = [self.drive.get_pose().x, self.drive.get_pose().y]
        mini = 100000

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            possible_sides = self.util.scoring_sides_red
        else:
            possible_sides = self.util.scoring_sides_blue

        lockout_tag = "0"
        for i in possible_sides:
            c = sqrt(((i[0] - pose[0]) * (i[0] - pose[0])) + ((i[1] - pose[1]) * (i[1] - pose[1])))
            if c < mini:
                mini = c
                lockout_tag = i[3]

        if lockout_tag in [7, 9, 11, 18, 22, 20]:
            return "algae_high"
        elif lockout_tag in [6, 8, 10, 17, 19, 21]:
            return "algae_low"
        else:
            return "algae_low"
