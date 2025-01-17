from commands2 import Subsystem
from wpilib import PowerDistribution, SmartDashboard, DriverStation


class UtilSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)

        self.scoring_location = 0
        # FORMAT: X, Y, ANGLE, LOCATION NAME, APRILTAG FOR SERVOING
        self.scoring_locations_blue = [
            [3.248, 4.516, 0.001, "Blue A", -10],
            [3.248, 4.182, 0.001, "Blue B", -10],
            [3.447, 3.163, 60, "Blue C", -10],
            [3.754, 3, 60, "Blue D", -10],
            [4.737, 2.7, 120, "Blue E", -10],
            [5.021, 2.871, 120, "Blue F", -10],
            [5.748, 3.562, 180.001, "Blue G", -10],
            [5.748, 3.9, 180.001, "Blue H", -10],
            [5.499, 3.539, 240, "Blue I", -10],
            [5.214, 5.058, 240, "Blue J", -10],
            [4.281, 5.328, 300, "Blue K", -10],
            [3.967, 5.186, 300, "Blue L", -10]
        ]
        self.scoring_locations_red = [
            [13.449, 3.560, 180.001, "Red A"],
            [13.449, 3.894, 180.001, "Red B"],
            [13.613, 4.095, 240, "Red C"],
            [13.348, 4.264, 240, "Red D"],
            [13.276, 4.593, 300, "Red E"],
            [13.003, 4.416, 300, "Red F"],
            [12.729, 4.473, 0.001, "Red G"],
            [12.729, 4.151, 0.001, "Red H"],
            [12.512, 3.975, 60, "Red I"],
            [12.810, 3.838, 60, "Red J"],
            [12.786, 3.565, 120, "Red K"],
            [13.083, 3.709, 120, "Red L"],
        ]
        self.scoring_sides_red = [
            [13.766, 4.031, [
                [13.449, 3.560, 180.001, "Red A Normal"],
                [13.449, 3.894, 180.001, "Red B Normal"],
                [13.324, 4.151, 180.001, "Red A Flipped"],
                [13.324, 4.489, 180.001, "Red B Flipped"],
            ]],
            [13.404, 4.609, [
                [13.613, 4.095, 240, "Red C Normal"],
                [13.348, 4.264, 240, "Red D Normal"],
                [13.107, 4.344, 240, "Red C Flipped"],
                [12.810, 4.473, 240, "Red D Flipped"],
            ]],
            [12.729, 4.585, [
                [13.316, 4.497, 120, "Red E Normal"],
                [13.043, 4.336, 120, "Red F Normal"],
                [12.810, 4.191, 120, "Red E Flipped"],
                [12.528, 4.031, 120, "Red F Flipped"],
            ]],
            [12.384, 4.015, [
                [12.729, 4.473, 0.001, "Red G Normal"],
                [12.729, 4.151, 0.001, "Red H Normal"],
                [12.753, 3.910, 0.001, "Red G Flipped"],
                [12.753, 3.541, 0.001, "Red H Flipped"],
            ]],
            [12.729, 3.412, [
                [12.512, 3.975, 60, "Red I Normal"],
                [12.810, 3.838, 60, "Red J Normal"],
                [13.043, 3.709, 60, "Red I Flipped"],
                [13.308, 3.541, 60, "Red J Flipped"],
            ]],
            [13.388, 3.482, [
                [12.786, 3.565, 300, "Red K Normal"],
                [13.083, 3.709, 300, "Red L Normal"],
                [13.340, 3.822, 300, "Red K Flipped"],
                [13.637, 4.007, 300, "Red L Flipped"],
            ]]
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
            SmartDashboard.putString("Scoring Location", self.scoring_locations_blue[self.scoring_location][3])
        SmartDashboard.putString("Scoring Setpoint", self.scoring_setpoints[self.scoring_setpoint])