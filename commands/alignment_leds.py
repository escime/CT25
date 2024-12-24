from commands2 import Command
from subsystems.ledsubsystem import LEDs
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain


class AlignmentLEDs(Command):

    def __init__(self, leds: LEDs, drive: CommandSwerveDrivetrain):
        super().__init__()
        self.leds = leds
        self.drive = drive
        self.addRequirements(leds)

    def initialize(self):
        self.leds.set_state("align")

    def execute(self):
        self.leds.set_misalignment(self.drive.get_auto_lookahead_heading(
                                [16.5, 5.53], 0.3), self.drive.get_pose().rotation().degrees())

    def end(self, interrupted: bool):
        self.leds.set_state("default")
