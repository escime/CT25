from commands2 import Command, WrapperCommand
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d
from math import pi, cos, sin
from generated.tuner_constants import TunerConstants
from wpilib import DriverStation
from wpimath.units import degreesToRadians


class PathfollowingEndpointClose(Command):
    def __init__(self, drive: CommandSwerveDrivetrain, endpoint: [float, float, float]):
        super().__init__()
        self.drive = drive
        self.endpoint = [endpoint[0], endpoint[1], Rotation2d.fromDegrees(endpoint[2])]

        self.drive_request = (swerve.requests.FieldCentricFacingAngle()
                              .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY))

        self.x_controller = PIDController(0.5, 0, 0, 0.04)
        self.y_controller = PIDController(0.5, 0, 0, 0.04)
        self.drive_request.heading_controller.setPID(5, 0, 0)
        self.drive_request.heading_controller.enableContinuousInput(0, -2 * pi)
        self.drive_request.heading_controller.setTolerance(0.05)

        # self.addRequirements(drive)

    def initialize(self):
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            x1 = ((self.endpoint[0] - 8.775) * cos(degreesToRadians(180))) - ((self.endpoint[1] - 4.007) * sin(degreesToRadians(180))) + 8.775
            y1 = ((self.endpoint[0] - 8.775) * sin(degreesToRadians(180))) + ((self.endpoint[1] - 4.007) * cos(degreesToRadians(180))) + 4.007
            self.endpoint[0] = x1
            self.endpoint[1] = y1
        else:
            self.endpoint[2] = Rotation2d.fromDegrees(self.endpoint[2].degrees() + 180)

        print("PATH CLOSING ENGAGED")

    def execute(self):
        current_pose = self.drive.get_pose()

        x_output = self.x_controller.calculate(current_pose.x, self.endpoint[0])
        y_output = self.y_controller.calculate(current_pose.y, self.endpoint[1])

        # self.drive.apply_request(lambda: (self.drive_request
        #                                   .with_velocity_x(-1 * x_output * TunerConstants.speed_at_12_volts)
        #                                   .with_velocity_y(-1 * y_output * TunerConstants.speed_at_12_volts)
        #                                   .with_target_direction(self.endpoint[2]))).schedule()

        self.drive.saved_request = (self.drive_request.with_velocity_x(-1 * x_output * TunerConstants.speed_at_12_volts)
                                    .with_velocity_y(-1 * y_output * TunerConstants.speed_at_12_volts)
                                    .with_target_direction(self.endpoint[2]))
        self.drive.endpoint = [self.endpoint[0], self.endpoint[1]]

    def isFinished(self) -> bool:
        # if self.endpoint[0] - 0.05 < self.drive.get_pose().x < self.endpoint[0] + 0.05 and self.endpoint[1] - 0.05 < self.drive.get_pose().y < self.endpoint[1] + 0.05:
        #     print("PATH CLOSING COMPLETE")
        #     return True
        # else:
        #     return False
        return True

    # def end(self, interrupted: bool):
    #     self.drive.apply_request(lambda: self.drive_request.with_velocity_x(0).with_velocity_y(0)).withTimeout(0.02).schedule()
