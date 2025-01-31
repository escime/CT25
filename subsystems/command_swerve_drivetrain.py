import math

from commands2 import Command, Subsystem, sysid
from phoenix6 import swerve, units, utils, SignalLogger
from typing import Callable, overload
from wpilib import DriverStation, Notifier, RobotController, SmartDashboard, Alert
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Rotation2d, Pose2d, Transform3d, Translation3d, Rotation3d, Pose3d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import PIDConstants, RobotConfig
from pathplannerlib.path import PathConstraints
from pathplannerlib.controller import PPHolonomicDriveController
from constants import AutoConstants
from wpimath.units import degreesToRadians, inchesToMeters
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from photonlibpy import photonCamera, photonPoseEstimator
from photonlibpy.simulation import VisionSystemSim, SimCameraProperties, PhotonCameraSim
from wpiutil import Sendable, SendableBuilder


class CommandSwerveDrivetrain(Subsystem, swerve.SwerveDrivetrain):
    """
    Class that extends the Phoenix 6 SwerveDrivetrain class and implements
    Subsystem so it can easily be used in command-based projects.
    """

    _SIM_LOOP_PERIOD: units.second = 0.005

    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    """Blue alliance sees forward as 0 degrees (toward red alliance wall)"""
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)
    """Red alliance sees forward as 180 degrees (toward blue alliance wall)"""

    auto_request = swerve.requests.ApplyRobotSpeeds()

    @overload
    def __init__(self, drivetrain_constants: swerve.SwerveDrivetrainConstants,
                 modules: list[swerve.SwerveModuleConstants]) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so user should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param driveTrainConstants: Drivetrain-wide constants for the swerve drive
        :type driveTrainConstants:  swerve.SwerveDrivetrainConstants
        :param modules:             Constants for each specific module
        :type modules:              list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
            self,
            drivetrain_constants: swerve.SwerveDrivetrainConstants,
            modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param driveTrainConstants: Drivetrain-wide constants for the swerve drive
        :type driveTrainConstants:  swerve.SwerveDrivetrainConstants
        :param modules:             Constants for each specific module
        :type modules:              list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
            self,
            drivetrain_constants: swerve.SwerveDrivetrainConstants,
            odometry_update_frequency: units.hertz,
            modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param driveTrainConstants:         Drivetrain-wide constants for the swerve drive
        :type driveTrainConstants:          swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
            self,
            drivetrain_constants: swerve.SwerveDrivetrainConstants,
            odometry_update_frequency: units.hertz,
            odometry_standard_deviation: tuple[float, float, float],
            vision_standard_deviation: tuple[float, float, float],
            modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param driveTrainConstants:         Drivetrain-wide constants for the swerve drive
        :type driveTrainConstants:          swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param odometry_standard_deviation: The standard deviation for odometry calculation
        :type odometry_standard_deviation:  tuple[float, float, float]
        :param vision_standard_deviation:   The standard deviation for vision calculation
        :type vision_standard_deviation:    tuple[float, float, float]
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    def __init__(
            self,
            drivetrain_constants: swerve.SwerveDrivetrainConstants,
            arg2=None,
            arg3=None,
            arg4=None,
            arg5=None,
    ):
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(self, drivetrain_constants, arg2, arg3, arg4, arg5)

        self.config = RobotConfig.fromGUISettings()

        self._sim_notifier: Notifier | None = None
        self._last_sim_time: units.second = 0.0

        self._has_applied_operator_perspective = False
        """Keep track if we've ever applied the operator perspective before or not"""

        if utils.is_simulation():
            self._start_sim_thread()

        self.target_lateral_offset = -1
        self.visible_tag = -1

        self.pathplanner_rotation_overridden = False
        self.configure_pathplanner()

        # Setup for velocity and acceleration calculations.
        self.lookahead_active = False
        self.loop_time = utils.get_current_time_seconds()
        self.vx_old = 0
        self.vy_old = 0
        self.omega_old = 0
        self.vx_new = 0
        self.vy_new = 0
        self.omega_new = 0
        self.ax = 0
        self.ay = 0
        self.alpha = 0
        self.vx_robot_new = 0
        self.vy_robot_new = 0
        self.omega_robot_new = 0
        self.vx_robot_old = 0
        self.vy_robot_old = 0
        self.omega_robot_old = 0
        self.ax_robot = 0
        self.ay_robot = 0
        self.alpha_robot = 0

        # Configure closed loop turning controller.
        self.clt_request = (
            swerve.requests.FieldCentricFacingAngle()
            # .with_deadband(self._max_speed * 0.1)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_desaturate_wheel_speeds(True)
        )
        self.clt_request.heading_controller.setPID(5, 0, 0)
        self.clt_request.heading_controller.enableContinuousInput(0, -2 * math.pi)
        self.clt_request.heading_controller.setTolerance(0.1)
        self.re_entered_clt = True
        self.target_direction = Rotation2d(0)

        # Configure persistent alerts.
        alert_photonvision_enabled = Alert("PhotonVision Simulation Enabled", Alert.AlertType.kWarning)

        self._translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self._steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self._rotation_characterization = swerve.requests.SysIdSwerveRotation()

        # Setup SYSID Routines.
        self.sys_id_routine_translation = sysid.SysIdRoutine(
            sysid.SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V to prevent brownout
                stepVoltage=4.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdTranslation_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            sysid.SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._translation_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        self.sys_id_routine_rotation = sysid.SysIdRoutine(
            sysid.SysIdRoutine.Config(
                # This is in radians per second², but SysId only supports "volts per second"
                rampRate=math.pi / 6,
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Use default timeout (10 s)
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            sysid.SysIdRoutine.Mechanism(
                lambda output: (
                    # output is actually radians per second, but SysId only supports "volts"
                    self.set_control(
                        self._rotation_characterization.with_rotational_rate(output)
                    ),
                    # also log the requested output for SysId
                    SignalLogger.write_double("Rotational_Rate", output),
                ),
                lambda log: None,
                self,
            ),
        )
        self.sys_id_routine_steer = self._sys_id_routine_steer = sysid.SysIdRoutine(
            sysid.SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            sysid.SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._steer_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )

        april_tag_field_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)
        cam1 = photonCamera.PhotonCamera("TAG_DETECT_FL")
        cam2 = photonCamera.PhotonCamera("TAG_DETECT_FR")
        robot_to_cam1 = Transform3d(Translation3d(inchesToMeters(9.625), inchesToMeters(9.625), inchesToMeters(8)),
                                    Rotation3d(0, degreesToRadians(-10), degreesToRadians(-45)))
        robot_to_cam2 = Transform3d(Translation3d(inchesToMeters(9.625), inchesToMeters(-9.625), inchesToMeters(8)),
                                    Rotation3d(0, degreesToRadians(-10), degreesToRadians(45)))

        photon_pose_cam1 = (
            photonPoseEstimator.PhotonPoseEstimator(april_tag_field_layout,
                                                    photonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
                                                    cam1,
                                                    robot_to_cam1))
        photon_pose_cam2 = (
            photonPoseEstimator.PhotonPoseEstimator(april_tag_field_layout,
                                                    photonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
                                                    cam2,
                                                    robot_to_cam2))

        self.photon_cam_array = [cam1, cam2]
        self.photon_pose_array = [photon_pose_cam1, photon_pose_cam2]

        self.used_tags = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]

        if utils.is_simulation():
            self.vision_sim = VisionSystemSim("main")
            self.vision_sim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape))
            camera_prop = SimCameraProperties()
            camera_prop.setCalibrationFromFOV(1280, 800, Rotation2d.fromDegrees(75))
            camera_prop.setCalibError(0.25, 0.08)
            camera_prop.setFPS(30)
            camera_prop.setAvgLatency(0.01)
            camera_prop.setLatencyStdDev(0.01)
            cam1_sim = PhotonCameraSim(cam1, camera_prop)
            cam2_sim = PhotonCameraSim(cam2, camera_prop)
            self.vision_sim.addCamera(cam1_sim, robot_to_cam1)
            self.vision_sim.addCamera(cam2_sim, robot_to_cam2)

        SmartDashboard.putData("Swerve Drive", SwerveDriveSendable(self))

    def apply_request(self, request: Callable[[], swerve.requests.SwerveRequest]) -> Command:
        """
        Returns a command that applies the specified control request to this swerve drivetrain.

        :param request: Lambda returning the request to apply
        :type request: Callable[[], swerve.requests.SwerveRequest]
        :returns: Command to run
        :rtype: Command
        """
        return self.run(lambda: self.set_control(request()))

    def periodic(self):
        # Periodically try to apply the operator perspective.
        # If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
        # This allows us to correct the perspective in case the robot code restarts mid-match.
        # Otherwise, only check and apply the operator perspective if the DS is disabled.
        # This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
        if not self._has_applied_operator_perspective or DriverStation.isDisabled():
            alliance_color = DriverStation.getAlliance()
            if alliance_color is not None:
                self.set_operator_perspective_forward(
                    self._RED_ALLIANCE_PERSPECTIVE_ROTATION
                    if alliance_color == DriverStation.Alliance.kRed
                    else self._BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self._has_applied_operator_perspective = True

        # Update robot velocity and acceleration.
        if self.lookahead_active:
            self.vel_acc_periodic()

        # Update Photonvision cameras.
        if self.photon_cam_array[0].isConnected():
            self.select_best_vision_pose()

        if utils.is_simulation():
            self.vision_sim.update(self.get_pose())

    def select_best_vision_pose(self) -> None:
        accepted_poses = []
        accepted_cameras = []
        final_pose = Pose3d(0, 0, 0, Rotation3d(0, 0, 0))
        final_timestamp = 0
        for i in range(0, len(self.photon_cam_array)):
            if self.photon_cam_array[i].getLatestResult().hasTargets():
                estimated_pose = self.photon_pose_array[i].update().estimatedPose
                if (0 < estimated_pose.x < 17.658 and 0 < estimated_pose.y < 8.131 and 0 <= estimated_pose.z <= 0.05 and
                        self.photon_cam_array[i].getLatestResult().getBestTarget().fiducialId in self.used_tags):
                    accepted_poses.append(estimated_pose)
                    accepted_cameras.append(self.photon_cam_array[i])

        if accepted_poses:
            min_ambiguity = 3940
            for j in range(0, len(accepted_cameras)):
                pose_ambiguity = accepted_cameras[j].getLatestResult().getBestTarget().getPoseAmbiguity()
                if pose_ambiguity < min_ambiguity:
                    min_ambiguity = pose_ambiguity
                    final_pose = accepted_poses[j]
                    final_timestamp = accepted_cameras[j].getLatestResult().getTimestampSeconds()

        if final_pose != Pose3d(0, 0, 0, Rotation3d(0, 0, 0)):
            SmartDashboard.putBoolean("Accepted new pose?", True)
            self.add_vision_measurement(final_pose.toPose2d(), final_timestamp, (0.4, 0.4, 999999999))
        else:
            SmartDashboard.putBoolean("Accepted new pose?", False)

    def set_used_tags(self, tags: str):
        if tags == "red_reef":
            self.used_tags = [6, 7, 8, 9, 10, 11]
        elif tags == "blue_reef":
            self.used_tags = [17, 18, 19, 20, 21, 22]
        elif tags == "border":
            self.used_tags = [1, 2, 3, 13, 12, 16]
        else:
            self.used_tags = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]

    def set_lookahead(self, on: bool) -> None:
        self.lookahead_active = on

    def vel_acc_periodic(self) -> None:
        """Calculates the instantaneous robot velocity and acceleration."""
        self.vx_new, self.vy_new, self.omega_new = self.get_field_relative_velocity()
        self.vx_robot_new, self.vy_robot_new, self.omega_robot_new = self.get_robot_relative_velocity()
        self.ax, self.ay, self.alpha = self.get_field_relative_acceleration([self.vx_new, self.vy_new, self.omega_new],
                                                                            [self.vx_old, self.vy_old, self.omega_old],
                                                                            utils.get_current_time_seconds() - self.loop_time)
        self.ax_robot, self.ay_robot, self.alpha_robot = (
            self.get_field_relative_acceleration([self.vx_robot_new, self.vy_robot_old, self.omega_robot_new],
                                                 [self.vx_robot_old, self.vy_robot_old, self.omega_robot_old],
                                                 utils.get_current_time_seconds() - self.loop_time))

        self.vx_old = self.vx_new
        self.vy_old = self.vy_new
        self.omega_old = self.omega_new
        self.vx_robot_old = self.vx_robot_new
        self.vy_robot_old = self.vy_robot_new
        self.omega_robot_old = self.omega_robot_new

        self.loop_time = utils.get_current_time_seconds()
        # SmartDashboard.putNumber("Robot Linear Speed", math.sqrt((self.vx_new * self.vx_new) + (self.vy_new * self.vy_new)))
        # SmartDashboard.putNumber("Robot Heading", self.get_pose().rotation().degrees())

    def get_field_relative_velocity(self) -> [float, float, float]:
        """Returns the instantaneous velocity of the robot."""
        return self.get_chassis_speeds().vx * self.get_pose().rotation().cos() - \
            self.get_chassis_speeds().vy * self.get_pose().rotation().sin(), \
            self.get_chassis_speeds().vy * self.get_pose().rotation().cos() + \
            self.get_chassis_speeds().vx * self.get_pose().rotation().sin(), self.get_chassis_speeds().omega

    def get_robot_relative_velocity(self) -> [float, float, float]:
        return self.get_chassis_speeds().vx, self.get_chassis_speeds().vy, self.get_chassis_speeds().omega

    def get_angular_velocity(self) -> float:
        """Returns the instantaneous angular velocity of the robot."""
        return self.get_chassis_speeds().omega

    def get_field_relative_acceleration(self, new_speed, old_speed, time: float) -> [float, float, float]:
        """Returns the instantaneous acceleration of the robot."""
        ax = (new_speed[0] - old_speed[0]) / time
        ay = (new_speed[1] - old_speed[1]) / time
        alpha = (new_speed[2] - old_speed[2]) / time

        if abs(ax) > 6.0:
            ax = 6.0 * math.copysign(1, ax)
        if abs(ay) > 6.0:
            ay = 6.0 * math.copysign(1, ay)
        if abs(alpha) > 4 * math.pi:
            alpha = 4 * math.pi * math.copysign(1, alpha)

        return ax, ay, alpha

    def _start_sim_thread(self):
        def _sim_periodic():
            current_time = utils.get_current_time_seconds()
            delta_time = current_time - self._last_sim_time
            self._last_sim_time = current_time

            # use the measured time delta, get battery voltage from WPILib
            self.update_sim_state(delta_time, RobotController.getBatteryVoltage())

        self._last_sim_time = utils.get_current_time_seconds()
        self._sim_notifier = Notifier(_sim_periodic)
        self._sim_notifier.startPeriodic(self._SIM_LOOP_PERIOD)

    def sys_id_translation_quasistatic(self, direction: sysid.SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_translation.quasistatic(direction)

    def sys_id_translation_dynamic(self, direction: sysid.SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_translation.dynamic(direction)

    def sys_id_rotation_quasistatic(self, direction: sysid.SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_rotation.quasistatic(direction)

    def sys_id_rotation_dynamic(self, direction: sysid.SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_rotation.dynamic(direction)

    def sys_id_steer_quasistatic(self, direction: sysid.SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_steer.quasistatic(direction)

    def sys_id_steer_dynamic(self, direction: sysid.SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_steer.dynamic(direction)

    def get_chassis_speeds(self):
        return AutoConstants.kinematics.toChassisSpeeds(self.get_state().module_states)

    def get_pose(self) -> Pose2d:
        """Returns the robot pose."""
        return self.get_state().pose

    def configure_pathplanner(self) -> None:
        """Configures all pathplanner settings."""
        AutoBuilder.configure(
            lambda: self.get_state().pose,
            self.reset_pose,
            lambda: self.get_state().speeds,
            lambda speeds, feedforwards: self.set_control(
                self.auto_request
                .with_speeds(speeds)
                .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)
            ),
            PPHolonomicDriveController(
                PIDConstants(AutoConstants.x_pid[0], AutoConstants.x_pid[1], AutoConstants.x_pid[2]),
                PIDConstants(AutoConstants.y_pid[0], AutoConstants.y_pid[1], AutoConstants.y_pid[2]),
                AutoConstants.speed_at_12_volts,
            ),
            self.config,
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed,
            # lambda: False,
            self
        )

        PPHolonomicDriveController.setRotationTargetOverride(self.pathplanner_rotation_override)

    def pathplanner_rotation_override(self) -> Rotation2d:
        """Provides the overridden heading in the event the override has been toggled. Returns None if override is
        disabled, which is the default."""
        if self.pathplanner_rotation_overridden == "goal":
            return Rotation2d.fromDegrees(self.get_goal_alignment_heading())
        elif self.pathplanner_rotation_overridden == "gp":
            return Rotation2d.fromDegrees(self.get_gp_alignment_heading())
        else:
            return None

    def get_gp_alignment_heading(self) -> float:
        return self.get_pose().rotation().degrees() + self.tx

    def get_goal_alignment_heading(self) -> float:
        """Returns the required target heading to point at a goal."""
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            return self.get_auto_lookahead_heading([16.5, 5.53], 0.3)
        else:
            return self.get_auto_lookahead_heading([0, 5.53], 0.3)

    def set_pathplanner_rotation_override(self, override: str) -> None:
        """Sets whether pathplanner uses an alternate heading controller."""
        self.pathplanner_rotation_overridden = override

    def get_auto_target_heading(self, target: [float, float]) -> float:
        """Acquires the target heading required to point at a goal."""
        current_pose = self.get_pose()
        return math.atan2(target[1] - current_pose.y, target[0] - current_pose.x) * 180 / math.pi

    def get_auto_lookahead_heading(self, target: [float, float], time_compensation: float) -> float:
        """Acquires the target heading required to point at a goal while the robot is in motion."""
        current_pose = self.get_pose()
        adjusted_pose = Pose2d(current_pose.x + self.vx_new * time_compensation,
                               current_pose.y + self.vy_new * time_compensation,
                               current_pose.rotation() + Rotation2d(self.omega_new * time_compensation))
        return math.atan2(target[1] - adjusted_pose.y, target[0] - adjusted_pose.x) * 180 / math.pi

    def pathfind_to_pose(self, target: [float, float, float]):
        """Command for pathfinding between current pose and a target pose in teleoperated."""
        target_pose = Pose2d(target[0], target[1], Rotation2d.fromDegrees(target[2]))
        constraints = PathConstraints(4, 4, 9.424, 12.567)

        return AutoBuilder.pathfindToPose(
            target_pose,
            constraints,
            goal_end_vel=0.0
        )

    def get_close_to_target(self, target: [float, float], good_range: float) -> bool:
        pose = [self.get_pose().x, self.get_pose().y]
        c = math.sqrt(((target[0] - pose[0]) * (target[0] - pose[0])) + ((target[1] - pose[1]) * (target[1] - pose[1])))
        return good_range >= c

    def reset_odometry(self):
        """Reset robot odometry at the Subwoofer."""
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.reset_pose(Pose2d(14.337, 4.020, Rotation2d.fromDegrees(180)))
            self.set_operator_perspective_forward(Rotation2d.fromDegrees(180))
        else:
            self.reset_pose(Pose2d(3.273, 4.020, Rotation2d.fromDegrees(0)))
            self.set_operator_perspective_forward(Rotation2d.fromDegrees(0))

    def reset_clt(self) -> None:
        self.re_entered_clt = True

    def drive_clt(self, x_speed: float, y_speed: float, turn_amount: float) -> swerve.requests:
        if self.re_entered_clt:
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                self.target_direction = Rotation2d.fromDegrees(self.get_pose().rotation().degrees() + 180)
            else:
                self.target_direction = self.get_pose().rotation()
            self.re_entered_clt = False
        else:
            self.target_direction = Rotation2d(self.target_direction.radians() + turn_amount * degreesToRadians(4))

        return (self.clt_request
                .with_velocity_x(x_speed)
                .with_velocity_y(y_speed)
                .with_target_direction(self.target_direction))

    def get_speed_callable_0(self) -> float:
        return self.get_state().module_states[0].speed

    def get_speed_callable_1(self) -> float:
        return self.get_state().module_states[1].speed

    def get_speed_callable_2(self) -> float:
        return self.get_state().module_states[1].speed

    def get_speed_callable_3(self) -> float:
        return self.get_state().module_states[1].speed

    def set_nothing(self) -> None:
        print("Wow, look at all that nothing.")


class SwerveDriveSendable(Sendable):

    def __init__(self, drive: CommandSwerveDrivetrain):
        super().__init__()
        self.drive = drive

    def initSendable(self, builder: SendableBuilder):
        builder.setSmartDashboardType("SwerveDrive")
        builder.addDoubleProperty("Front Left Angle", self.drive.get_state().module_states[0].angle.radians, self.drive.set_nothing)
        builder.addDoubleProperty("Front Left Velocity", self.drive.get_speed_callable_0, self.drive.set_nothing)
        builder.addDoubleProperty("Front Right Angle", self.drive.get_state().module_states[1].angle.radians, self.drive.set_nothing)
        builder.addDoubleProperty("Front Right Velocity", self.drive.get_speed_callable_1, self.drive.set_nothing)
        builder.addDoubleProperty("Back Left Angle", self.drive.get_state().module_states[2].angle.radians, self.drive.set_nothing)
        builder.addDoubleProperty("Back Left Velocity", self.drive.get_speed_callable_2, self.drive.set_nothing)
        builder.addDoubleProperty("Back Right Angle", self.drive.get_state().module_states[3].angle.radians, self.drive.set_nothing)
        builder.addDoubleProperty("Back Right Velocity", self.drive.get_speed_callable_3, self.drive.set_nothing)
        builder.addDoubleProperty("Robot Angle", self.drive.get_pose().rotation().radians, self.drive.set_nothing)
