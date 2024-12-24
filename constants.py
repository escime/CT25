"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""
from wpimath.units import inchesToMeters
from phoenix6 import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics


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

