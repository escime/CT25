from commands2 import Command, SequentialCommandGroup, InterruptionBehavior
from wpilib import Timer, DriverStation
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.utilsubsystem import UtilSubsystem
from phoenix6 import swerve
from generated.tuner_constants import TunerConstants
from subsystems.elevatorandarm import ReZeroTorque
from commands.set_elevator_and_arm import SetElevatorAndArm
from math import sqrt

from subsystems.elevatorandarm import ElevatorAndArmSubsystem


class Score(Command):
    def __init__(self, elevator_and_arm: ElevatorAndArmSubsystem, drive: CommandSwerveDrivetrain, util: UtilSubsystem, timer: Timer):
        super().__init__()
        self.elevator_and_arm = elevator_and_arm
        self.timer = timer
        self.drive = drive
        self.util = util

        self.forward_request = (swerve.requests.RobotCentric()
                                .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
                                .with_velocity_y(0))

        self.locked_in = False
        self.time_in = 10000000000000

        self.addRequirements(elevator_and_arm)

    def initialize(self):
        self.time_in = 10000000000000
        self.locked_in = False
        if "stage" or "score" in self.elevator_and_arm.get_arm_state() and "manual" not in self.elevator_and_arm.get_elevator_state():
            if "left" in self.elevator_and_arm.get_arm_state():
                if "L4" in self.elevator_and_arm.get_elevator_state():
                    arm_target = "score_left_L4"
                    self.drive.apply_request(lambda: self.forward_request.with_velocity_x(
                        0.2 * TunerConstants.speed_at_12_volts)).withTimeout(0.25).schedule()

                else:
                    arm_target = "score_left"
            else:
                if "L4" in self.elevator_and_arm.get_elevator_state():
                    arm_target = "score_right_L4"
                    self.drive.apply_request(
                        lambda: self.forward_request.with_velocity_x(
                            -0.2 * TunerConstants.speed_at_12_volts)).withTimeout(0.25).schedule()
                else:
                    arm_target = "score_right"
            self.elevator_and_arm.set_arm_state(arm_target)
            if "L4" in self.elevator_and_arm.get_elevator_state():
                self.elevator_and_arm.set_elevator_state("L4_scoring")
                self.elevator_and_arm.intake.set(0.35)
            elif "L3" in self.elevator_and_arm.get_elevator_state():
                self.elevator_and_arm.set_elevator_state("L3_scoring")
                self.elevator_and_arm.intake.set(0.35)
            elif "L2" in self.elevator_and_arm.get_elevator_state():
                self.elevator_and_arm.set_elevator_state("L2_scoring")
                self.elevator_and_arm.intake.set(0.35)

    def execute(self):
        if self.elevator_and_arm.get_arm_at_target() and self.elevator_and_arm.get_elevator_at_target() and not self.locked_in:
            self.elevator_and_arm.intake.set(0.35)
            self.locked_in = True
            self.time_in = self.timer.get()

        if self.locked_in and self.timer.get() - self.time_in > 0.05:
            self.elevator_and_arm.set_arm_state("stow")

    def isFinished(self) -> bool:
        # print(self.elevator_and_arm.get_arm_at_target())
        return self.timer.get() - self.time_in > 0.1

    def end(self, interrupted: bool):
        self.elevator_and_arm.intake.set(0)
        if self.util.algae_mode:
            SetElevatorAndArm(self.get_algae_height(), self.elevator_and_arm, self.drive).schedule()
        else:
            SequentialCommandGroup(
                SetElevatorAndArm("stow", self.elevator_and_arm, self.drive).withTimeout(1.5),
                ReZeroTorque(self.elevator_and_arm).withInterruptBehavior(InterruptionBehavior.kCancelSelf).withTimeout(0.1)
            ).schedule()

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
