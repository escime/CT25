from commands2 import Command
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.units import rotationsToDegrees
from math import atan2, pi


class ProfiledTarget(Command):
    def __init__(self, drive: CommandSwerveDrivetrain, target: [float, float]):
        super().__init__()
        self.drive = drive
        self.target = target

        self.rotation_request = (swerve.requests.RobotCentric()
                                 .with_velocity_y(0)
                                 .with_velocity_x(0)
                                 .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY))

        self.rotation_controller = ProfiledPIDController(0.13, 0, 0,
                                                         TrapezoidProfile.Constraints(rotationsToDegrees(0.75),
                                                                                      rotationsToDegrees(0.1)))
        self.rotation_controller.enableContinuousInput(-180, 180)
        self.rotation_controller.setTolerance(2)

    def execute(self):
        current_pose = self.drive.get_pose()
        rotation_target = atan2(self.target[1] - current_pose.y, self.target[0] - current_pose.x) * 180 / pi

        rotate_output = self.rotation_controller.calculate(current_pose.rotation().degrees(), rotation_target)

        self.drive.apply_request(lambda: (self.rotation_request
                                          .with_rotational_rate(rotate_output))).schedule()

        if self.rotation_controller.atSetpoint():
            print("PROFILED CONTROLLER AT SETPOINT")
        elif not self.rotation_controller.atSetpoint():
            print("PROFILED CONTROLLER NOT AT SETPOINT")

    def end(self, interrupted: bool):
        self.drive.apply_request(lambda: self.rotation_request).withTimeout(0.02).schedule()
