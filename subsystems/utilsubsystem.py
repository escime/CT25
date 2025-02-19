from commands2 import Subsystem
from wpilib import PowerDistribution, SmartDashboard, DriverStation


class UtilSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)

        self.auto_start_time = 0

        self.scoring_location = 0
        self.algae_mode = False
        # FORMAT: X, Y, ANGLE, LOCATION NAME, APRILTAG FOR SERVOING
        self.scoring_sides_red = [
            [13.766, 4.031, [
                [13.324, 4.151, 180.001, "Red A Flipped"],
                [13.324, 4.489, 180.001, "Red B Flipped"],
                [13.449, 3.560, 180.001, "Red A Normal"],
                [13.449, 3.894, 180.001, "Red B Normal"],
            ], 7],
            [13.404, 4.609, [
                [13.107, 4.344, 240, "Red C Flipped"],
                [12.810, 4.473, 240, "Red D Flipped"],
                [13.613, 4.095, 240, "Red C Normal"],
                [13.348, 4.264, 240, "Red D Normal"],
            ], 8],
            [12.729, 4.585, [
                [12.810, 4.191, 120, "Red E Flipped"],
                [12.528, 4.031, 120, "Red F Flipped"],
                [13.316, 4.497, 120, "Red E Normal"],
                [13.043, 4.336, 120, "Red F Normal"],
            ], 9],
            [12.384, 4.015, [
                [12.753, 3.910, 0.001, "Red G Flipped"],
                [12.753, 3.541, 0.001, "Red H Flipped"],
                [12.729, 4.473, 0.001, "Red G Normal"],
                [12.729, 4.151, 0.001, "Red H Normal"],
            ], 10],
            [12.729, 3.412, [
                [13.043, 3.709, 60, "Red I Flipped"],
                [13.308, 3.541, 60, "Red J Flipped"],
                [12.512, 3.975, 60, "Red I Normal"],
                [12.810, 3.838, 60, "Red J Normal"],
            ], 11],
            [13.388, 3.482, [
                # [13.340, 3.822, 300, "Red K Flipped"],
                # [13.637, 4.007, 300, "Red L Flipped"],
                # [12.786, 3.565, 300, "Red K Normal"],
                # [13.083, 3.709, 300, "Red L Normal"],
                [13.304, 3.909, 300, "Red K Flipped"],
                [13.633, 3.976, 300, "Red L Flipped"],
                [12.737, 3.641 , 300, "Red K Normal"],
                [13.177, 3.551, 300, "Red L Normal"],
            ], 6]
        ]
        self.scoring_sides_blue = [
            [3.795, 4.018, [
                [4.242, 3.900, 0.001, "Blue A Flipped"],
                [4.215, 3.555, 0.001, "Blue B Flipped"],
                [4.204, 4.482, 0.001, "Blue A Normal"],
                [4.204, 4.148, 0.001, "Blue B Normal"],
            ], 18],
            [4.140, 3.426, [
                [4.468, 3.679, 60, "Blue C Flipped"],
                [4.743, 3.523, 60, "Blue D Flipped"],
                [3.935, 4.024, 60, "Blue C Normal"],
                [4.221, 3.873, 60, "Blue D Normal"],
            ], 17],
            [4.829, 3.437, [
                [4.748, 3.835, 300, "Blue E Flipped"],
                [5.055, 3.981, 300, "Blue F Flipped"],
                [4.226, 3.560, 300, "Blue E Normal"],
                [4.501, 3.711, 300, "Blue F Normal"],
            ], 22],
            [5.169, 4.029, [
                [4.765, 4.164, 180.001, "Blue G Flipped"],
                [4.765, 4.498, 180.001, "Blue H Flipped"],
                [4.765, 3.582, 180.001, "Blue G Normal"],
                [4.765, 3.889, 180.001, "Blue H Normal"],
            ], 21],
            [4.824, 4.627, [
                [4.506, 4.341, 240, "Blue I Flipped"],
                [4.231, 4.482, 240, "Blue J Flipped"],
                [4.996, 3.991, 240, "Blue I Normal"],
                [4.748, 4.207, 240, "Blue J Normal"],
            ], 20],
            [4.145, 4.611, [
                [4.242, 4.180, 120, "Blue K Flipped"],
                [3.951, 4.045, 120, "Blue L Flipped"],
                [4.775, 4.482, 120, "Blue K Normal"],
                [4.458, 4.341, 120, "Blue L Normal"],
            ], 19]
        ]

        self.feeder_sides_red = [
            [17.5, 8.5, 55],
            [17.5, 0, 125]
        ]
        self.feeder_sides_blue = [
            [0.683, 7.245, 305],
            [0.683, 0.758, 235]
        ]

        self.scoring_setpoint = 0
        self.scoring_setpoints = ["stow", "max"]

    def toggle_channel(self, on: bool) -> None:
        self.pdh.setSwitchableChannel(on)

    def cycle_scoring_setpoints(self, cycle_amount: int) -> None:
        if cycle_amount + self.scoring_setpoint >= len(self.scoring_setpoints):
            self.scoring_setpoint = 0
        elif cycle_amount + self.scoring_setpoint < 0:
            self.scoring_setpoint = len(self.scoring_setpoints) - 1
        else:
            self.scoring_setpoint += cycle_amount

    def change_algae_mode(self, algae_mode_enabled: bool) -> None:
        self.algae_mode = algae_mode_enabled
        SmartDashboard.putBoolean("Algae Mode?", algae_mode_enabled)

    # def periodic(self) -> None:
    #     SmartDashboard.putString("Scoring Setpoint", self.scoring_setpoints[self.scoring_setpoint])
