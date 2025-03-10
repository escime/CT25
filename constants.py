"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""
from wpimath.units import inchesToMeters, lbsToKilograms
from phoenix6 import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from math import pi


class OIConstants:
    kDriverControllerPort = 0
    kOperatorControllerPort = 1


class LEDConstants:
    port = 0
    strip_length = 28


class AutoConstants:
    # Copy these values from TunerConstants.
    _front_left_x_pos: units.meter = inchesToMeters(9.625)
    _front_left_y_pos: units.meter = inchesToMeters(9.625)
    _front_right_x_pos: units.meter = inchesToMeters(9.625)
    _front_right_y_pos: units.meter = inchesToMeters(-9.625)
    _back_left_x_pos: units.meter = inchesToMeters(-9.625)
    _back_left_y_pos: units.meter = inchesToMeters(9.625)
    _back_right_x_pos: units.meter = inchesToMeters(-9.625)
    _back_right_y_pos: units.meter = inchesToMeters(-9.625)

    # Math for based on copied units from Tuner.
    _front_left_translation = Translation2d(_front_left_x_pos, _front_left_y_pos)
    _front_right_translation = Translation2d(_front_right_x_pos, _front_right_y_pos)
    _back_left_translation = Translation2d(_back_left_x_pos, _back_left_y_pos)
    _back_right_translation = Translation2d(_back_right_x_pos, _back_right_y_pos)
    kinematics = SwerveDrive4Kinematics(_front_left_translation, _front_right_translation, _back_left_translation,
                                        _back_right_translation)

    drive_base_radius = Translation2d(_front_left_x_pos, _front_left_y_pos).norm()
    speed_at_12_volts: units.meters_per_second = 4.73

    drive_gear_ratio = 6.746031746031747

    # PID Constants for PathPlanner
    x_pid = [5, 0, 0]
    y_pid = [5, 0, 0]


class ElevatorConstants:
    can_ids = [33, 32]  # The main motor should be listed first. The rest may be listed in any order.

    carriage_weight = lbsToKilograms(10)
    min_height_in = 0
    max_height_in = 40
    min_height_m = inchesToMeters(min_height_in)
    max_height_m = inchesToMeters(max_height_in)

    drum_diameter_in = 2  # 14T sprocket
    drum_diameter_m = inchesToMeters(drum_diameter_in)
    state_values = {"stow": 0, "max": 6.45,
                    "L1": 0,
                    "L2_scoring": 0.5,
                    "L2": 1.25,
                    "L3_scoring": 2.3,
                    "L3": 3.25,
                    "L4_scoring": 4.8,
                    "L4": 6.4,
                    "algae_high": 4.3,
                    "algae_low": 2,
                    "net": 6.4}

    supply_current_limit = 80
    use_supply_current_limit = True

    gearbox_ratio = 4

    mm_cruise_velocity = 10 #  23  # rot/s  # was 7 for testing
    mm_acceleration = 20 # 100 # rot/s^2  # was 15 for testing
    mm_jerk = 1000  # rot/s^3

    kg = 0.33
    ks = 0.1
    kv = 3.11
    ka = 0.04
    kp = 80
    ki = 0
    kd = 0

    elevator_angle_degrees = 90

    elevator_at_target_threshold = 0.02
    elevator_upper_limit = max_height_in / (drum_diameter_in * pi)
    elevator_lower_limit = 0 / (drum_diameter_in * pi)


class ArmConstants:
    state_values = {"stow": 0,
                    "stage_left": 0.07,
                    "score_left": 0.14,
                    "score_left_L4": 0.155, # 0.14
                    "stage_right": -0.07,
                    "score_right": -0.14,
                    "score_right_L4": -0.155,  # -0.14
                    "algae_left": 0.20,
                    "algae_right": -0.20}
    wrist_can_id = 34
    intake_channel = 2
    score_speed = 0.75
    supply_current_limit = 40
    use_supply_current_limit = True
    gearbox_ratio = 50
    arm_at_target_threshold = 0.01

    mm_cruise_velocity = 2.5 # was 0.5 for testing
    mm_acceleration = 1.25 # 5
    mm_jerk = 100

    kg = 0.25
    ks = 0.25
    kv = 0.71
    ka = 0.01
    kp = 110
    ki = 0.001
    kd = 0

    arm_weight = lbsToKilograms(10)
    arm_length = inchesToMeters(20)


class IntakeConstants:
    intake_state_values = {"stow": 0.1, "stow_algae": 0.2, "intake_coral": 0.73, "intake_algae": 0.29, "score_coral": 0.15, "score_algae": 0.07, "climbing": 0.3}
    wheel_speed_values = {"stow": 0, "stow_algae": 0.25, "intake_coral": -12, "intake_algae": 12, "score_coral": 0, "score_algae": -12, "climbing": 0}
    wheel_can_id = 30
    arm_can_id = 31
    gearbox_ratio = 12

    mm_cruise_velocity = 2
    mm_acceleration = 10
    mm_jerk = 100

    kg = 1.25
    ks = 0.25
    kv = 0.32
    ka = 0.06
    kp = 80
    ki = 0
    kd = 0


class ClimberConstants:
    climber_can_id = 36
    gearbox_ratio = 100
