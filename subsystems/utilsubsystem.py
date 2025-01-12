from commands2 import Subsystem
from wpilib import PowerDistribution, SmartDashboard, DriverStation


class UtilSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)

        self.scoring_location = 0
        # FORMAT: X, Y, ANGLE, LOCATION NAME, APRILTAG FOR SERVOING
        self.scoring_locations_red = [
            [13.714, 3.539, 180.001, "Red A", -10],
            [13.714, 3.887, 120, "Red B", -10],
            [10.929, 2.8, 60, "Red Source", -10]
        ]
        self.scoring_locations_blue = [
            [16.5 - self.scoring_locations_red[0][0], 4.111, 0.001, "Blue Podium", 4],
            [16.5 - self.scoring_locations_red[2][0], 2.8, 300, "Blue Source", 5],
            [16.5 - self.scoring_locations_red[1][0], 5.364, 240, "Blue Amp", 6]
        ]

        self.scoring_setpoint = 0
        self.scoring_setpoints = ["stow", "max"]

    def toggle_channel(self, on: bool) -> None:
        self.pdh.setSwitchableChannel(on)

    def cycle_scoring_locations(self, cycle_amount: int) -> None:
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            if cycle_amount + self.scoring_location >= len(self.scoring_locations_red):
                self.scoring_location = 0
            elif cycle_amount + self.scoring_location < 0:
                self.scoring_location = len(self.scoring_locations_red) - 1
            else:
                self.scoring_location += cycle_amount
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            if cycle_amount + self.scoring_location >= len(self.scoring_locations_blue):
                self.scoring_location = 0
            elif cycle_amount + self.scoring_location < 0:
                self.scoring_location = len(self.scoring_locations_blue) - 1
            else:
                self.scoring_location += cycle_amount

    def cycle_scoring_setpoints(self, cycle_amount: int) -> None:
        if cycle_amount + self.scoring_setpoint >= len(self.scoring_setpoints):
            self.scoring_setpoint = 0
        elif cycle_amount + self.scoring_setpoint < 0:
            self.scoring_setpoint = len(self.scoring_setpoints) - 1
        else:
            self.scoring_setpoint += cycle_amount

    def periodic(self) -> None:
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            SmartDashboard.putString("Scoring Location", self.scoring_locations_red[self.scoring_location][3])
        else:
            SmartDashboard.putString("Scoring Location", self.scoring_locations_red[self.scoring_location][3])
        SmartDashboard.putString("Scoring Setpoint", self.scoring_setpoints[self.scoring_setpoint])