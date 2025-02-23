from commands2 import Command, button

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.utilsubsystem import UtilSubsystem
from phoenix6 import swerve
from generated.tuner_constants import TunerConstants
from math import sqrt, pi
from wpilib import DriverStation
from wpimath.geometry import Rotation2d


class CoralStationSimple(Command):
    def __init__(self, drive: CommandSwerveDrivetrain, util: UtilSubsystem,
                 joystick: button.CommandXboxController):
        super().__init__()
        self.drive = drive
        self.joystick = joystick
        self.util = util
        self.flipped = False

        self.drive_request = (swerve.requests.FieldCentricFacingAngle()
                              .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY))

        self.drive_request.heading_controller.setPID(5, 0, 0)
        self.drive_request.heading_controller.enableContinuousInput(0, -2 * pi)
        self.drive_request.heading_controller.setTolerance(0.1)

        self.heading = Rotation2d(0)

        # self.addRequirements(self.elevator)
        # self.addRequirements(self.arm)

    def initialize(self):
        self.heading = self.get_closest_target()
        self.drive.set_used_tags("all")
        # self.arm.set_state(self.util.scoring_setpoints[self.util.scoring_setpoint])

    def execute(self):
        y_move = self.joystick.getLeftY() * -1
        x_move = self.joystick.getLeftX() * -1

        self.drive.apply_request(lambda: (self.drive_request
                                          .with_velocity_x(y_move * TunerConstants.speed_at_12_volts * 0.35)
                                          .with_velocity_y(x_move * TunerConstants.speed_at_12_volts * 0.35)
                                          .with_target_direction(self.heading))).schedule()

    def end(self, interrupted: bool):
        self.drive.apply_request(lambda: self.drive_request).withTimeout(0.01).schedule()
        self.drive.set_used_tags("all")
        # self.arm.set_state("stow")

    def get_closest_target(self) -> [float, float, float, float, float]:
        pose = [self.drive.get_pose().x, self.drive.get_pose().y]
        mini = 100000

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            possible_sides = self.util.feeder_sides_red
        else:
            possible_sides = self.util.feeder_sides_blue

        for i in possible_sides:
            c = sqrt(((i[0] - pose[0]) * (i[0] - pose[0])) + ((i[1] - pose[1]) * (i[1] - pose[1])))
            if c < mini:
                mini = c
                mini_target = [i[0], i[1], i[2]]

        if mini_target == [17.5, 8.5, 55]:
            heading = Rotation2d.fromDegrees(55 - 90)
        elif mini_target == [17.5, 0, 125]:
            heading = Rotation2d.fromDegrees(125 + 90)
        elif mini_target == [0.683, 7.245, 305]:
            heading = Rotation2d.fromDegrees(-55 + 90 + 180)
        elif mini_target == [0.683, 0.758, 235]:
            heading = Rotation2d.fromDegrees(-125 - 90 + 180)
        else:
            heading = Rotation2d.fromDegrees(0)

        return heading
