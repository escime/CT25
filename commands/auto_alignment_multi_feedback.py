from commands2 import Command, button

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.utilsubsystem import UtilSubsystem
from subsystems.elevatorandarm import ElevatorAndArmSubsystem
from phoenix6 import swerve
from wpimath.controller import PIDController
from generated.tuner_constants import TunerConstants
from wpimath.units import degreesToRadians, radiansToDegrees, metersToInches
from math import sqrt, pow, cos, sin, atan2
from wpilib import DriverStation, SmartDashboard


class AutoAlignmentMultiFeedback(Command):
    def __init__(self, drive: CommandSwerveDrivetrain, util: UtilSubsystem,
                 joystick: button.CommandXboxController, side: str):
        super().__init__()
        self.drive = drive
        # self.elevator = elevator
        self.joystick = joystick
        self.util = util
        self.flipped = True
        self.side = side

        self.forward_request = (swerve.requests.RobotCentric()
                                .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
                                .with_velocity_y(0))

        self.rotate_controller = PIDController(0.1, 0, 0, 0.04)
        self.rotate_controller.enableContinuousInput(-180, 180)
        self.y_controller = PIDController(0.65, 0.01, 0, 0.04)
        # self.closing_controller = PIDController(0.1, 0, 0, 0.04)
        self.target = [0, 0]

        # self.addRequirements(self.elevator)
        # self.addRequirements(self.arm)

    def initialize(self):
        self.target = self.get_closest_target()
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.drive.set_used_tags("red_reef")
        else:
            self.drive.set_used_tags("blue_reef")
        # self.arm.set_state(self.util.scoring_setpoints[self.util.scoring_setpoint])

    def execute(self):
        x_move = self.joystick.getLeftY() * -1

        current_pose = self.drive.get_pose()
        a = self.target[0] - current_pose.x
        b = self.target[1] - current_pose.y

        theta = radiansToDegrees(atan2(b, a))

        if theta < 0:
            theta = 360 + theta

        if self.flipped:
            theta += 180
            x_move = x_move * -1

        current_heading = current_pose.rotation().degrees()
        if current_heading < 0:
            current_heading = 360 + current_heading

        rotate_output = self.rotate_controller.calculate(current_heading, theta)
        y_output = self.y_controller.calculate(self.get_vector_to_line(current_pose, self.target[2]), 0)

        if -0.005 < y_output < 0.005:
            y_output = 0

        # if self.joystick.rightTrigger().getAsBoolean():
        #     y_output = self.joystick.getLeftX() * 0.25
        #     if not self.flipped:
        #         y_output *= -1
        #     rotate_output = 0

        self.drive.apply_request(lambda: (self.forward_request
                                          .with_velocity_x(x_move * TunerConstants.speed_at_12_volts * 0.35)
                                          .with_rotational_rate(rotate_output)
                                          .with_velocity_y(y_output * TunerConstants.speed_at_12_volts))).schedule()

    def end(self, interrupted: bool):
        self.drive.apply_request(lambda: self.forward_request).withTimeout(0.02).schedule()
        self.drive.set_used_tags("all")
        # self.arm.set_state("stow")

    def get_closest_target_coordinates(self, current_pose, alpha) -> [float, float]:
        """When given x, output y."""
        y1 = self.target[1]
        x1 = self.target[0]
        c = 2

        y2 = y1 + c * sin(degreesToRadians(alpha))
        x2 = x1 + c * cos(degreesToRadians(alpha))
        m = (y2 - y1) / (x2 - x1)
        b = y2 - (m * x2)

        xmin = (current_pose.x + (m * current_pose.y) - (b * m)) / (pow(m, 2) + 1)
        ymin = ((-1 * m * (-1 * current_pose.x - (m * current_pose.y))) + b) / (pow(m, 2) + 1)

        if alpha > 180:
            ymin = current_pose.y - (ymin - current_pose.y)
        return xmin, ymin

    def get_point_on_line(self, x: float, alpha) -> float:
        y2 = self.target[1]
        x2 = self.target[0]
        c = 2

        y1 = y2 - c * cos(degreesToRadians(alpha))
        x1 = x2 - c * sin(degreesToRadians(alpha))
        m = (y2 - y1) / (x2 - x1)
        b = y2 - (m * x2)

        return (m * x) + b

    def get_distance_to_line(self, current_pose, alpha) -> float:
        y1 = self.target[1]
        x1 = self.target[0]
        c = 2

        y2 = y1 + c * sin(degreesToRadians(alpha))
        x2 = x1 + c * cos(degreesToRadians(alpha))
        m = (y2 - y1) / (x2 - x1)
        b = y2 - (m * x2)

        return abs((-1 * m * current_pose.x) + current_pose.y - b) / sqrt(pow(-1 * m, 2) + 1)

    def get_vector_to_line(self, current_pose, alpha):
        xmin, ymin = self.get_closest_target_coordinates(current_pose, alpha)

        SmartDashboard.putString("Perceived Alignment Error",
                                 str(metersToInches(self.get_distance_to_line(current_pose, alpha))) + "in")
        if not self.flipped:
            if ymin > current_pose.y:
                return -1 * self.get_distance_to_line(current_pose, alpha)
            elif ymin < current_pose.y:
                return self.get_distance_to_line(current_pose, alpha)
            else:
                return 0
        else:
            if ymin < current_pose.y:
                return -1 * self.get_distance_to_line(current_pose, alpha)
            elif ymin > current_pose.y:
                return self.get_distance_to_line(current_pose, alpha)
            else:
                return 0

    def get_closest_target(self) -> [float, float, float, float, float]:
        pose = [self.drive.get_pose().x, self.drive.get_pose().y]
        mini = 100000

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            possible_sides = self.util.scoring_sides_red
        else:
            possible_sides = self.util.scoring_sides_blue

        for i in possible_sides:
            c = sqrt(((i[0] - pose[0]) * (i[0] - pose[0])) + ((i[1] - pose[1]) * (i[1] - pose[1])))
            if c < mini:
                mini = c
                mini_target = [i[0], i[1], i[2]]

        if self.side == "left":
            if self.check_to_use():
                SmartDashboard.putString("Auto Scoring Setpoint", mini_target[2][0][-1])
                self.flipped = True
                return mini_target[2][0]
            else:
                SmartDashboard.putString("Auto Scoring Setpoint", mini_target[2][2][-1])
                self.flipped = False
                return mini_target[2][2]
        else:
            if self.check_to_use():
                SmartDashboard.putString("Auto Scoring Setpoint", mini_target[2][1][-1])
                self.flipped = True
                return mini_target[2][1]
            else:
                SmartDashboard.putString("Auto Scoring Setpoint", mini_target[2][3][-1])
                self.flipped = False
                return mini_target[2][3]

    def check_to_use(self):
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            if self.drive.get_pose().x >= 13.067:
                return -90 < self.drive.get_pose().rotation().degrees() < 90
            else:
                return not -90 < self.drive.get_pose().rotation().degrees() < 90
        else:
            if self.drive.get_pose().x <= 4.484:
                return not -90 < self.drive.get_pose().rotation().degrees() < 90
            else:
                return -90 < self.drive.get_pose().rotation().degrees() < 90
