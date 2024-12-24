from commands2 import Command

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.ledsubsystem import LEDs
from wpilib import DriverStation, SendableChooser
from math import cos, sin, sqrt, copysign
from wpimath.units import degreesToRadians
from wpimath.geometry import Pose2d, Translation2d, Rotation2d


class AutoAlignmentLEDs(Command):
    def __init__(self, drive: CommandSwerveDrivetrain, leds: LEDs, selector: SendableChooser):
        super().__init__()
        self.leds = leds
        self.drive = drive
        self.start = "A"
        self.selector = selector

        self.addRequirements(leds)

        self.start_coords = [0, 0, 0]
        self.resetter = False

    def initialize(self):
        self.leds.set_state("align")
        self.start = self.selector.getSelected()
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            alliance = "red"
        else:
            alliance = "blue"
        if self.start == "B" and alliance == "red":
            self.start_coords = [15.18, 5.55, 180, 90.0001]
        elif self.start == "A" and alliance == "red":
            self.start_coords = [15.81, 6.68, 120, 150.0001]
        elif self.start == "C" and alliance == "red":
            self.start_coords = [15.81, 4.44, 240, 30.0001]
        elif self.start == "B" and alliance == "blue":
            self.start_coords = [1.34, 5.55, 0, 270.0001]
        elif self.start == "A" and alliance == "blue":
            self.start_coords = [0.74, 6.68, 60, 210.0001]
        elif self.start == "C" and alliance == "blue":
            self.start_coords = [0.74, 4.44, -60, 330.0001]
        else:
            self.start_coords = [15.18, 5.55, 180, 90.0001]

        self.drive.reset_pose(Pose2d(Translation2d(self.start_coords[0], self.start_coords[1]),
                              Rotation2d.fromDegrees(self.start_coords[2])))

    def execute(self):
        misalignment = self.get_vector_to_line(self.drive.get_pose(), self.start_coords[3])
        if abs(misalignment) * 100 > 179:
            misalignment = copysign(179, misalignment)
        else:
            misalignment = misalignment * 100
        self.leds.set_misalignment(0, misalignment)
        if -0.07 < misalignment / 100 < 0.07 and not self.resetter:
            self.leds.set_state("default")
            self.resetter = True
        elif (misalignment >= 0.07 or misalignment / 100 <= -0.07) and self.resetter:
            self.resetter = False
            self.leds.set_state("align")

    def end(self, interrupted: bool):
        self.leds.set_state("default")

    def get_closest_target_coordinates(self, current_pose, alpha) -> [float, float]:
        """When given x, output y."""
        y2 = self.start_coords[1]
        x2 = self.start_coords[0]
        c = 2

        y1 = y2 - c * cos(degreesToRadians(alpha))
        x1 = x2 - c * sin(degreesToRadians(alpha))
        m = (y2 - y1) / (x2 - x1)
        b = y2 - (m * x2)

        xmin = (current_pose.x + (m * current_pose.y) - (b * m)) / (pow(m, 2) + 1)
        ymin = ((-1 * m * (-1 * current_pose.x - (m * current_pose.y))) + b) / (pow(m, 2) + 1)

        return xmin, ymin

    def get_distance_to_line(self, current_pose, alpha) -> float:
        y2 = self.start_coords[1]
        x2 = self.start_coords[0]
        c = 2

        y1 = y2 - c * cos(degreesToRadians(alpha))
        x1 = x2 - c * sin(degreesToRadians(alpha))
        m = (y2 - y1) / (x2 - x1)
        b = y2 - (m * x2)

        return abs((-1 * m * current_pose.x) + current_pose.y - b) / sqrt(pow(m, 2) + pow(1, 2))

    def get_vector_to_line(self, current_pose, alpha):
        xmin, ymin = self.get_closest_target_coordinates(current_pose, alpha)

        if current_pose.x < xmin:
            return -1 * self.get_distance_to_line(current_pose, alpha)
        elif current_pose.x > xmin:
            return self.get_distance_to_line(current_pose, alpha)
        else:
            return 0
