from commands2 import Command, WrapperCommand
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve
from wpilib import SmartDashboard, Timer
from constants import AutoConstants
from wpimath.units import degreesToRadians, rotationsToRadians, metersToInches


class WheelRadiusCalculator(Command):

    def __init__(self, drive: CommandSwerveDrivetrain, timer: Timer):
        super().__init__()
        self.drive = drive
        self.timer = timer

        self.drive_request = (swerve.requests.RobotCentric()
                              .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY))

        # self.addRequirements(drive)

        self.fl = 0
        self.fr = 0
        self.bl = 0
        self.br = 0
        self.start_angle = 0
        self.start_time = 0

    def initialize(self):
        self.start_time = self.timer.get()

        self.drive.get_module(0).drive_motor.set_position(0)
        self.drive.get_module(1).drive_motor.set_position(0)
        self.drive.get_module(2).drive_motor.set_position(0)
        self.drive.get_module(3).drive_motor.set_position(0)

        self.fl = abs(self.drive.get_module(0).drive_motor.get_position().value_as_double)
        self.fr = abs(self.drive.get_module(1).drive_motor.get_position().value_as_double)
        self.bl = abs(self.drive.get_module(2).drive_motor.get_position().value_as_double)
        self.br = abs(self.drive.get_module(3).drive_motor.get_position().value_as_double)

        self.start_angle = self.drive.get_pose().rotation().degrees()

    def execute(self):
        self.drive.apply_request(lambda: (self.drive_request
                                          .with_velocity_x(0)
                                          .with_velocity_y(0)
                                          .with_rotational_rate(0.1 * rotationsToRadians(0.75)))).schedule()

    def isFinished(self) -> bool:
        if self.start_angle - 0.1 < self.drive.get_pose().rotation().degrees() <= self.start_angle + 0.1 and self.timer.get() - 2 > self.start_time:
            return True
        else:
            return False

    def end(self, interrupted: bool):
        self.drive.apply_request(lambda: self.drive_request.with_rotational_rate(0)).withTimeout(
            0.02).schedule()

        if not interrupted:
            fl_dist = abs(self.drive.get_module(0).drive_motor.get_position().value_as_double) - self.fl
            fr_dist = abs(self.drive.get_module(1).drive_motor.get_position().value_as_double) - self.fr
            bl_dist = abs(self.drive.get_module(2).drive_motor.get_position().value_as_double) - self.bl
            br_dist = abs(self.drive.get_module(3).drive_motor.get_position().value_as_double) - self.br

            fl_n = fl_dist * AutoConstants.drive_gear_ratio
            fr_n = fr_dist * AutoConstants.drive_gear_ratio
            bl_n = bl_dist * AutoConstants.drive_gear_ratio
            br_n = fl_dist * AutoConstants.drive_gear_ratio

            avg_dist = (fl_dist + fr_dist + bl_dist + br_dist) / 4

            SmartDashboard.putNumber("Average Distance Traveled", avg_dist)

            avg_n = (fl_n + fr_n + bl_n + br_n) / 4

            wheel_radius = metersToInches(AutoConstants.drive_base_radius) / avg_n

            SmartDashboard.putNumber("Average Wheel Radius", wheel_radius)
