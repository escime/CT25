from commands2 import Command
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve
from generated.tuner_constants import TunerConstants
from wpimath.units import rotationsToRadians
from wpilib import Timer


class Baseline(Command):

    def __init__(self, drive: CommandSwerveDrivetrain, timer: Timer):
        super().__init__()
        self.drive = drive
        # self.addRequirements(drive)
        self.timer = timer
        self.drive_forward = swerve.requests.RobotCentric()\
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)\
            .with_velocity_x(0.85 * TunerConstants.speed_at_12_volts)\
            .with_velocity_y(0)\
            .with_rotational_rate(0)
        self.drive_reverse = swerve.requests.RobotCentric()\
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)\
            .with_velocity_x(-1 * 0.85 * TunerConstants.speed_at_12_volts) \
            .with_velocity_y(0) \
            .with_rotational_rate(0)
        self.drive_left = swerve.requests.RobotCentric()\
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)\
            .with_velocity_y(0.85 * TunerConstants.speed_at_12_volts) \
            .with_velocity_x(0) \
            .with_rotational_rate(0)
        self.drive_right = swerve.requests.RobotCentric()\
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)\
            .with_velocity_y(-1 * 0.85 * TunerConstants.speed_at_12_volts) \
            .with_velocity_x(0) \
            .with_rotational_rate(0)
        self.rotate_ccw = swerve.requests.RobotCentric()\
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)\
            .with_rotational_rate(rotationsToRadians(0.75)) \
            .with_velocity_x(0) \
            .with_velocity_y(0)
        self.rotate_cw = swerve.requests.RobotCentric()\
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)\
            .with_rotational_rate(rotationsToRadians(-1 * 0.75)) \
            .with_velocity_x(0) \
            .with_velocity_y(0)

        self.drive_currents = [[], [], [], []]
        self.steer_currents = [[], [], [], []]

        self.start_time = self.timer.get()

    def initialize(self):
        self.start_time = self.timer.get()

    def execute(self):
        if not self.check_time(2):
            self.drive.apply_request(lambda: self.drive_forward).schedule()
        if not self.check_time(4) and self.check_time(2):
            self.drive.apply_request(lambda: self.drive_reverse).schedule()
        if not self.check_time(6) and self.check_time(4):
            self.drive.apply_request(lambda: self.drive_left).schedule()
        elif not self.check_time(8) and self.check_time(6):
            self.drive.apply_request(lambda: self. drive_right).schedule()
        elif not self.check_time(10) and self.check_time(8):
            self.drive.apply_request(lambda: self.rotate_cw).schedule()
        elif not self.check_time(12) and self.check_time(10):
            self.drive.apply_request(lambda: self.rotate_ccw).schedule()

        for i in range(0, 4):
            self.drive_currents[i].append(self.drive.modules[i].drive_motor.get_stator_current().value_as_double)
            self.steer_currents[i].append(self.drive.modules[i].steer_motor.get_stator_current().value_as_double)

    def isFinished(self) -> bool:
        if self.check_time(12):
            return True
        else:
            return False

    def end(self, interrupted: bool):
        self.drive.apply_request(lambda: swerve.requests.SwerveDriveBrake()).schedule()
        print("Front Left Drive Max Current Draw: " + str(max(self.drive_currents[0])))
        print("Front Left Steer Max Current Draw: " + str(max(self.steer_currents[0])))
        print("Front Right Drive Max Current Draw: " + str(max(self.drive_currents[1])))
        print("Front Right Steer Max Current Draw: " + str(max(self.steer_currents[1])))
        print("Back Left Drive Max Current Draw: " + str(max(self.drive_currents[2])))
        print("Back Left Steer Max Current Draw: " + str(max(self.steer_currents[2])))
        print("Back Right Drive Max Current Draw: " + str(max(self.drive_currents[3])))
        print("Back Right Steer Max Current Draw: " + str(max(self.steer_currents[3])))

    def check_time(self, time: float) -> bool:
        if self.timer.get() - time > self.start_time:
            return True
        else:
            return False
