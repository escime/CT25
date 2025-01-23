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
    strip_length = 25


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

    # PID Constants for PathPlanner
    x_pid = [5, 0, 0]
    y_pid = [5, 0, 0]


class ElevatorConstants:
    can_ids = [32, 33]  # The main motor should be listed first. The rest may be listed in any order.

    carriage_weight = lbsToKilograms(10)
    min_height_in = 0
    max_height_in = 40
    min_height_m = inchesToMeters(min_height_in)
    max_height_m = inchesToMeters(max_height_in)

    drum_diameter_in = 1.685  # 14T sprocket
    drum_diameter_m = inchesToMeters(drum_diameter_in)
    state_values = {"stow": 0, "max": max_height_in / (drum_diameter_in * pi),
                    "L1": 5 / (drum_diameter_in * pi),
                    "L2": 10 / (drum_diameter_in * pi),
                    "L3": 20 / (drum_diameter_in * pi),
                    "L4": 30 / (drum_diameter_in * pi)}

    supply_current_limit = 80
    use_supply_current_limit = True

    gearbox_ratio = 4

    mm_cruise_velocity = 24  # rot/s
    mm_acceleration = 650  # rot/s^2
    mm_jerk = 1000  # rot/s^3

    kg = 0.09
    ks = 0.1
    kv = 3.11
    ka = 0
    kp = 60
    ki = 0
    kd = 0

    elevator_angle_degrees = 90

    elevator_at_target_threshold = 0.05
    elevator_upper_limit = max_height_in / (drum_diameter_in * pi)
    elevator_lower_limit = 0 / (drum_diameter_in * pi)


class ArmConstants:
    state_values = {"stow": 0.25, "stage_right": 0.2, "score_right": 0.1, "stage_left": 0.3, "score_left": 0.4}
    wrist_can_id = 34
    score_speed = 0.25
    supply_current_limit = 40
    use_supply_current_limit = False
    gearbox_ratio = 20.8
    arm_at_target_threshold = 0.01

    mm_cruise_velocity = 0.5
    mm_acceleration = 5
    mm_jerk = 100

    kg = 0.66
    ks = 0.25
    kv = 0.4
    ka = 0.02
    kp = 90
    ki = 0
    kd = 0

    arm_weight = lbsToKilograms(10)
    arm_length = inchesToMeters(20)

class IntakeConstants:
    intake_state_values = {"stow": 0, "intake_coral": 0.25, "intake_algae": 0.15, "score_coral": 0.1}
    wheel_speed_values = {"stow": 0, "intake_coral": 12, "intake_algae": -12, "score_coral": -2.4}
    wheel_can_id = 30
    arm_can_id = 31
    gearbox_ratio = 12

    mm_cruise_velocity = 0.5
    mm_acceleration = 5
    mm_jerk = 100

    kg = 1.02
    ks = 0.25
    kv = 0.23
    ka = 0.02
    kp = 80
    ki = 0
    kd = 0

class ClimberConstants:
    climber_can_id = 36
    gearbox_ratio = 100