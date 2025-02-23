from commands2 import Command, WrapperCommand
from wpilib import DriverStation
from subsystems.elevatorandarm import ElevatorAndArmSubsystem
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain


class SetElevatorAndArm(Command):
    def __init__(self, setpoint: str, elevator_and_arm: ElevatorAndArmSubsystem, drive: CommandSwerveDrivetrain):
        super().__init__()
        self.setpoint = setpoint
        self.elevator_and_arm = elevator_and_arm
        self.drive = drive

        self.wait_for = "none"
        self.arm_target = "none"

        self.addRequirements(elevator_and_arm)

    def initialize(self):
        if self.setpoint == "stow":
            self.elevator_and_arm.set_accel_limit(1)
        if self.setpoint == "L1":
            self.elevator_and_arm.set_accel_limit(1)
        if self.setpoint == "L2":
            self.elevator_and_arm.set_accel_limit(1)
        if self.setpoint == "L3":
            self.elevator_and_arm.set_accel_limit(0.7)
        if self.setpoint == "L4":
            self.elevator_and_arm.set_accel_limit(0.5)

        if self.setpoint != "stow" and self.setpoint != "net":
            if self.setpoint == "L1" or self.setpoint == "L2" or self.setpoint == "L3" or self.setpoint == "L4":
                self.arm_target = "stage_"
            else:
                self.arm_target = "algae_"

            if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                if self.drive.get_pose().x > 13:
                    if -90 < self.drive.get_pose().rotation().degrees() < 90:
                        self.arm_target += "left"
                    else:
                        self.arm_target += "right"
                elif self.drive.get_pose().x < 13:
                    if -90 < self.drive.get_pose().rotation().degrees() < 90:
                        self.arm_target += "right"
                    else:
                        self.arm_target += "left"
            else:
                if self.drive.get_pose().x > 4:
                    if -90 < self.drive.get_pose().rotation().degrees() < 90:
                        self.arm_target += "left"
                    else:
                        self.arm_target += "right"
                elif self.drive.get_pose().x < 4:
                    if -90 < self.drive.get_pose().rotation().degrees() < 90:
                        self.arm_target += "right"
                    else:
                        self.arm_target += "left"
        else:
            self.arm_target = "stow"

        self.elevator_and_arm.set_elevator_state(self.setpoint)
        if self.elevator_and_arm.get_arm_state() != "stow":
            self.elevator_and_arm.set_arm_state("stow")

    def isFinished(self) -> bool:
        return self.elevator_and_arm.get_elevator_at_target()

    def end(self, interrupted: bool):
        if not interrupted:
            self.elevator_and_arm.set_arm_state(self.arm_target)
