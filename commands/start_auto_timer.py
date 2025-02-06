from commands2 import Command
from wpilib import Timer
from subsystems.utilsubsystem import UtilSubsystem


class StartAutoTimer(Command):
    def __init__(self, util: UtilSubsystem, timer: Timer):
        super().__init__()
        self.util = util
        self.timer = timer

    def initialize(self):
        self.util.auto_start_time = self.timer.get()

    def isFinished(self) -> bool:
        return True
