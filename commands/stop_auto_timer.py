from commands2 import Command
from wpilib import Timer, SmartDashboard
from subsystems.utilsubsystem import UtilSubsystem


class StopAutoTimer(Command):
    def __init__(self, util: UtilSubsystem, timer: Timer):
        super().__init__()
        self.util = util
        self.timer = timer
        self.completed_time = 0

    def initialize(self):
        self.completed_time = self.timer.get() - self.util.auto_start_time

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool):
        SmartDashboard.putString("Auto Time Elapsed", str(self.completed_time))
