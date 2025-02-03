from commands2.cmd import run, runOnce, runEnd
import wpilib.simulation
from commands2 import Command, button, SequentialCommandGroup, ParallelCommandGroup, ParallelRaceGroup, sysid, \
    InterruptionBehavior, ParallelDeadlineGroup, WaitCommand

from constants import OIConstants
from subsystems.climbersubsystem import Climber
from subsystems.intakesubsystem import Intake
from subsystems.ledsubsystem import LEDs
from subsystems.utilsubsystem import UtilSubsystem
from subsystems.elevatorandarm import ElevatorAndArmSubsystem
from wpilib import SmartDashboard, SendableChooser, DriverStation, DataLogManager, Timer, Alert
from pathplannerlib.auto import NamedCommands, PathPlannerAuto, AutoBuilder
from wpinet import PortForwarder

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6 import swerve, SignalLogger
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

from math import pi

from commands.baseline import Baseline
from commands.check_drivetrain import CheckDrivetrain
from commands.alignment_leds import AlignmentLEDs
from commands.profiled_target import ProfiledTarget
from commands.auto_alignment_multi_feedback import AutoAlignmentMultiFeedback
from commands.set_elevator_and_arm import SetElevatorAndArm
from commands.score import Score
from commands.collect_from_cs import Collect
from commands.auto_set_elevator_and_arm import AutoSetElevatorAndArm
from commands.coral_station_alignment import CoralStationAlignment
from commands.score_attempt import ScoreAttempt
from commands.wheel_radius_calculator import WheelRadiusCalculator


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # Start master timer. ------------------------------------------------------------------------------------------
        self.timer = Timer()
        self.timer.start()

        # Configure button to enable robot logging.
        self.logging_button = SmartDashboard.putBoolean("Logging Enabled?", False)

        # Disable automatic ctre logging
        SignalLogger.enable_auto_logging(False)

        # Configure system logging. ------------------------------------------------------------------------------------
        self.alert_logging_enabled = Alert("Robot Logging is Enabled", Alert.AlertType.kWarning)
        self.alert_limelight = Alert("Limelight ports forwarded", Alert.AlertType.kWarning)
        if wpilib.RobotBase.isReal():
            if SmartDashboard.getBoolean("Logging Enabled?", False) is True:
                DataLogManager.start()
                DriverStation.startDataLog(DataLogManager.getLog(), True)
                SignalLogger.start()
                self.alert_logging_enabled.set(True)
            else:
                SignalLogger.stop()
            for port in range(5800, 5810):
                PortForwarder.getInstance().add(port, "10.39.40.11", port)
            for port in range(5800, 5810):
                PortForwarder.getInstance().add(port+10, "10.39.40.12", port)
            self.alert_limelight.set(True)
        else:
            SignalLogger.stop()

        # Startup subsystems. ------------------------------------------------------------------------------------------
        self.leds = LEDs(self.timer)
        self.util = UtilSubsystem()
        self.elevator_and_arm = ElevatorAndArmSubsystem()
        self.intake_arm = Intake()
        self.climber_arm = Climber()

        # Setup driver & operator controllers. -------------------------------------------------------------------------
        self.driver_controller = button.CommandXboxController(OIConstants.kDriverControllerPort)
        self.operator_controller = button.CommandXboxController(OIConstants.kOperatorControllerPort)
        DriverStation.silenceJoystickConnectionWarning(True)
        self.test_bindings = False

        # Configure drivetrain settings. -------------------------------------------------------------------------------
        self._max_speed = TunerConstants.speed_at_12_volts  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(0.75)  # 3/4 of a rotation per second max angular velocity

        self._logger = Telemetry(self._max_speed)

        self.drivetrain = TunerConstants.create_drivetrain()

        self._drive = (
            swerve.requests.FieldCentric()  # I want field-centric
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(self._max_angular_rate * 0.1)  # Add a 10% deadband
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_desaturate_wheel_speeds(True)
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._hold_heading = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(self._max_speed * 0.1)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_desaturate_wheel_speeds(True)
        )
        self._hold_heading.heading_controller.setPID(5, 0, 0)
        self._hold_heading.heading_controller.enableContinuousInput(0, -2 * pi)
        self._hold_heading.heading_controller.setTolerance(0.1)

        # Register commands for PathPlanner. ---------------------------------------------------------------------------
        self.registerCommands()

        # Set up new autonomous selection structure
        # self.m_auto_start_location = SendableChooser()
        # self.m_auto_start_location.setDefaultOption("A", "A")
        # self.m_auto_num_gp = SendableChooser()
        # self.m_auto_num_gp.setDefaultOption("1", "1")
        # SmartDashboard.putData("Auto Start Selector", self.m_auto_start_location)
        # SmartDashboard.putData("Auto GP Num Selector", self.m_auto_num_gp)

        SmartDashboard.putBoolean("Misalignment Indicator Active?", False)
        SmartDashboard.putNumber("Misalignment Angle", 0)

        # Setup for all event-trigger commands. ------------------------------------------------------------------------
        # self.configureTriggersSmartDash()
        self.configure_test_bindings()
        self.configure_triggers()

        # Setup autonomous selector on the dashboard. ------------------------------------------------------------------
        # self.m_chooser = SendableChooser()
        # self.auto_names = ["Test", "Baseline", "CheckDrivetrain", "BuildPlay", "1-Score4"]
        # self.m_chooser.setDefaultOption("DoNothing", "DoNothing")
        # for x in self.auto_names:
        #     self.m_chooser.addOption(x, x)
        # SmartDashboard.putData("Auto Select", self.m_chooser)
        self.m_chooser = AutoBuilder.buildAutoChooser("DoNothing")
        SmartDashboard.putData("Auto Select", self.m_chooser)

    def configure_triggers(self) -> None:
        self.drivetrain.setDefaultCommand(  # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self.driver_controller.getLeftY() * self._max_speed * self.elevator_and_arm.get_accel_limit())
                    .with_velocity_y(-self.driver_controller.getLeftX() * self._max_speed * self.elevator_and_arm.get_accel_limit())
                    .with_rotational_rate(-self.driver_controller.getRightX() * self._max_angular_rate * self.elevator_and_arm.get_accel_limit())
                )
            )
        )

        # self.drivetrain.setDefaultCommand(  # Drivetrain will execute this command periodically
        #     self.drivetrain.apply_request(
        #         lambda: (
        #             self._drive.with_velocity_x(
        #                 -self.driver_controller.getLeftY() * self._max_speed)
        #             .with_velocity_y(
        #                 -self.driver_controller.getLeftX() * self._max_speed)
        #             .with_rotational_rate(
        #                 -self.driver_controller.getRightX() * self._max_angular_rate)
        #         )
        #     )
        # )

        # Slow mode
        self.driver_controller.rightTrigger().whileTrue(
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self.driver_controller.getLeftY() * self._max_speed * 0.5)
                    .with_velocity_y(
                        -self.driver_controller.getLeftX() * self._max_speed * 0.5)
                    .with_rotational_rate(
                        -self.driver_controller.getRightX() * self._max_angular_rate * 0.5)
                )
            )
        )

        # self.drivetrain.setDefaultCommand(
        #     SequentialCommandGroup(
        #             runOnce(lambda: self.drivetrain.reset_clt(), self.drivetrain),
        #             self.drivetrain.apply_request(
        #                 lambda: (
        #                     self.drivetrain.drive_clt(
        #                         self.driver_controller.getLeftY() * self._max_speed * -1 * self.elevator_and_arm.get_accel_limit(),
        #                         self.driver_controller.getLeftX() * self._max_speed * -1 * self.elevator_and_arm.get_accel_limit(),
        #                         self.driver_controller.getRightX() * -1 * self.elevator_and_arm.get_accel_limit()
        #                     )
        #                 )
        #             )
        #         )
        #     )

        # Reset pose.
        self.driver_controller.y().and_(lambda: not self.test_bindings).onTrue(
            runOnce(lambda: self.drivetrain.reset_odometry(), self.drivetrain).ignoringDisable(True))

        # Auto selecting auto alignment for Reef scoring positions.
        self.driver_controller.x().and_(lambda: not self.test_bindings).whileTrue(
            AutoAlignmentMultiFeedback(self.drivetrain, self.util, self.driver_controller, "left"))
        self.driver_controller.b().and_(lambda: not self.test_bindings).whileTrue(
            AutoAlignmentMultiFeedback(self.drivetrain, self.util, self.driver_controller, "right"))

        # Auto selecting auto alignment for Coral Stations.
        self.driver_controller.a().and_(lambda: not self.test_bindings).whileTrue(
            ParallelDeadlineGroup(
                CoralStationAlignment(self.drivetrain, self.util, self.driver_controller),
                SetElevatorAndArm("stow", self.elevator_and_arm, self.drivetrain)
                .andThen(Collect(self.elevator_and_arm)))
        )

        # Toggle scoring state
        self.driver_controller.leftBumper().and_(lambda: not self.test_bindings).onTrue(
            ScoreAttempt(self.elevator_and_arm)
        )

        # Score Coral
        self.driver_controller.leftTrigger().and_(lambda: not self.test_bindings).and_(
            lambda: not self.util.algae_mode).onTrue(
            Score(self.elevator_and_arm, self.timer)
            .andThen(SetElevatorAndArm("stow", self.elevator_and_arm, self.drivetrain))
        )

        # Change between CORAL and ALGAE scoring modes.
        self.operator_controller.rightTrigger(0.1).and_(lambda: not self.test_bindings).onTrue(
            runOnce(lambda: self.util.change_algae_mode(False), self.util).ignoringDisable(True)
        )
        self.operator_controller.leftTrigger(0.1).and_(lambda: not self.test_bindings).onTrue(
            runOnce(lambda: self.util.change_algae_mode(True), self.util).ignoringDisable(True)
        )

        # Intake coral
        self.operator_controller.leftBumper().and_(lambda: not self.test_bindings).whileTrue(
            Collect(self.elevator_and_arm)
        )

        # Human player LEDs
        self.operator_controller.start().and_(lambda: not self.test_bindings).onTrue(
            SequentialCommandGroup(
                runOnce(lambda: self.leds.set_flash_color_rate(15), self.leds),
                runOnce(lambda: self.leds.set_flash_color_color([255, 255, 255]), self.leds),
                runOnce(lambda: self.leds.set_state("flash_color"), self.leds)
            ).ignoringDisable(True)
        ).onFalse(
            runOnce(lambda: self.leds.set_state("default"), self.leds).ignoringDisable(True)
        )

        # Cycle through scoring set points.
        # self.driver_controller.povLeft().and_(lambda: not self.test_bindings).onTrue(
        #     runOnce(lambda: self.util.cycle_scoring_setpoints(1), self.util).ignoringDisable(True)
        # )
        # self.driver_controller.povRight().and_(lambda: not self.test_bindings).onTrue(
        #     runOnce(lambda: self.util.cycle_scoring_setpoints(-1), self.util).ignoringDisable(True)
        # )

        # Manually control the elevator.
        self.operator_controller.povUp().whileTrue(
            run(lambda: self.elevator_and_arm.set_elevator_manual(0.15 * 12), self.elevator_and_arm)
        ).onFalse(
            runOnce(lambda: self.elevator_and_arm.set_elevator_manual_off(), self.elevator_and_arm)
        )
        self.operator_controller.povDown().whileTrue(
            run(lambda: self.elevator_and_arm.set_elevator_manual(0.05 * -1 * 12), self.elevator_and_arm)
        ).onFalse(
            runOnce(lambda: self.elevator_and_arm.set_elevator_manual_off(), self.elevator_and_arm)
        )
        self.operator_controller.povLeft().whileTrue(
            run(lambda: self.elevator_and_arm.set_arm_manual(0.1 * 12), self.elevator_and_arm)
        ).onFalse(
            runOnce(lambda: self.elevator_and_arm.set_arm_manual_off(), self.elevator_and_arm)
        )
        self.operator_controller.povRight().whileTrue(
            run(lambda: self.elevator_and_arm.set_arm_manual(0.1 * -1 * 12), self.elevator_and_arm)
        ).onFalse(
            runOnce(lambda: self.elevator_and_arm.set_arm_manual_off(), self.elevator_and_arm)
        )

        # Set the Elevator and Arm.
        self.operator_controller.rightBumper().and_(lambda: not self.test_bindings).and_(lambda: not self.util.algae_mode).onTrue(
            SetElevatorAndArm("L4", self.elevator_and_arm, self.drivetrain)
        )
        self.operator_controller.y().and_(lambda: not self.test_bindings).and_(lambda: not self.util.algae_mode).onTrue(
            SetElevatorAndArm("L3", self.elevator_and_arm, self.drivetrain)
        )
        self.operator_controller.x().and_(lambda: not self.test_bindings).and_(lambda: not self.util.algae_mode).onTrue(
            SetElevatorAndArm("L2", self.elevator_and_arm, self.drivetrain)
        )
        self.operator_controller.b().and_(lambda: not self.test_bindings).and_(lambda: not self.util.algae_mode).onTrue(
            SetElevatorAndArm("L1", self.elevator_and_arm, self.drivetrain)
        )
        self.operator_controller.rightBumper().and_(lambda: not self.test_bindings).and_(lambda: self.util.algae_mode).onTrue(
            SetElevatorAndArm("net", self.elevator_and_arm, self.drivetrain)
        )
        self.operator_controller.y().and_(lambda: not self.test_bindings).and_(lambda: self.util.algae_mode).onTrue(
            SetElevatorAndArm("algae_high", self.elevator_and_arm, self.drivetrain)
        )
        self.operator_controller.b().and_(lambda: not self.test_bindings).and_(lambda: self.util.algae_mode).onTrue(
            SetElevatorAndArm("algae_low", self.elevator_and_arm, self.drivetrain)
        )
        self.operator_controller.a().and_(lambda: not self.test_bindings).onTrue(
            SetElevatorAndArm("stow", self.elevator_and_arm, self.drivetrain)
        )

        # Manually control the climber
        self.operator_controller.axisGreaterThan(4, 0.2).and_(lambda: not self.test_bindings).onTrue(
            SequentialCommandGroup(
                runOnce(lambda: self.intake_arm.set_state("climbing"), self.intake_arm),
                runOnce(lambda: self.leds.set_state("rainbow"), self.leds),
                run(lambda: self.climber_arm.set_climber_manual(self.operator_controller.getRightX() * -1 * 12),
                    self.climber_arm)
            )
        ).onFalse(
            runOnce(lambda: self.climber_arm.set_climber_manual(0), self.climber_arm)
        )
        self.operator_controller.axisLessThan(4, -0.2).and_(lambda: not self.test_bindings).onTrue(
            SequentialCommandGroup(
                runOnce(lambda: self.intake_arm.set_state("climbing"), self.intake_arm),
                runOnce(lambda: self.leds.set_state("rainbow"), self.leds),
                run(lambda: self.climber_arm.set_climber_manual(self.operator_controller.getRightX() * -1 * 12),
                    self.climber_arm)
            )
        ).onFalse(
            runOnce(lambda: self.climber_arm.set_climber_manual(0), self.climber_arm)
        )
        
        # Intake controls.
        (self.operator_controller.axisGreaterThan(1, 0.1).and_(lambda: not self.test_bindings)
            .and_(lambda: not self.util.algae_mode).onTrue(
            runOnce(lambda: self.intake_arm.set_state("intake_coral"), self.intake_arm)
        ).onFalse(
            runOnce(lambda: self.intake_arm.set_state("stow"), self.intake_arm)
        ))
        self.operator_controller.axisGreaterThan(1, 0.1).and_(lambda: not self.test_bindings).and_(lambda: self.util.algae_mode).onTrue(
            runOnce(lambda: self.intake_arm.set_state("intake_algae"), self.intake_arm)
        ).onFalse(
            runOnce(lambda: self.intake_arm.set_state("stow"), self.intake_arm)
        )
        self.driver_controller.rightBumper().and_(lambda: not self.test_bindings).and_(lambda: not self.util.algae_mode).onTrue(
            runOnce(lambda: self.intake_arm.set_state("score_coral"), self.intake_arm)
        ).onFalse(
            runOnce(lambda: self.intake_arm.set_state("stow"), self.intake_arm)
        )
        self.driver_controller.rightBumper().and_(lambda: not self.test_bindings).and_(
            lambda: self.util.algae_mode).onTrue(
            runOnce(lambda: self.intake_arm.set_state("score_algae"), self.intake_arm)
        ).onFalse(
            runOnce(lambda: self.intake_arm.set_state("stow"), self.intake_arm)
        )

        # Debug Mode Toggle
        self.driver_controller.start().and_(lambda: not self.test_bindings).onTrue(
            SequentialCommandGroup(
                runOnce(lambda: self.intake_arm.set_debug_mode(), self.intake_arm),
                runOnce(lambda: self.elevator_and_arm.set_debug_mode(), self.elevator_and_arm),
                runOnce(lambda: self.climber_arm.set_debug_mode(), self.climber_arm)
            ).ignoringDisable(True)
        )

        # Coral acquired light
        button.Trigger(lambda: self.elevator_and_arm.get_coral_sensors() and DriverStation.isTeleop()).onTrue(
            SequentialCommandGroup(
                runOnce(lambda: self.leds.set_flash_color_rate(10), self.leds),
                runOnce(lambda: self.leds.set_flash_color_color([0, 255, 0]), self.leds),
                runOnce(lambda: self.leds.set_state("flash_color"), self.leds),
                WaitCommand(2),
                runOnce(lambda: self.leds.set_state("gp_held"), self.leds)
            ).ignoringDisable(True)
        ).onFalse(
            SequentialCommandGroup(
                runOnce(lambda: self.leds.set_flash_color_rate(10), self.leds),
                runOnce(lambda: self.leds.set_flash_color_color([255, 0, 0]), self.leds),
                runOnce(lambda: self.leds.set_state("flash_color"), self.leds),
                WaitCommand(0.5),
                runOnce(lambda: self.leds.set_state("default"), self.leds)
            ).ignoringDisable(True)
        )

        # Algae acquired light
        # button.Trigger(lambda: self.intake_arm.get_sensor_on() and DriverStation.isTeleop()).onTrue(
        #     SequentialCommandGroup(
        #         runOnce(lambda: self.leds.set_flash_color_rate(10), self.leds),
        #         runOnce(lambda: self.leds.set_flash_color_color([0, 0, 255]), self.leds),
        #         runOnce(lambda: self.leds.set_state("flash_color"), self.leds),
        #         WaitCommand(2),
        #         runOnce(lambda: self.leds.set_state("gp_held"), self.leds)
        #     ).ignoringDisable(True)
        # ).onFalse(
        #     SequentialCommandGroup(
        #         runOnce(lambda: self.leds.set_flash_color_rate(10), self.leds),
        #         runOnce(lambda: self.leds.set_flash_color_color([255, 153, 0]), self.leds),
        #         runOnce(lambda: self.leds.set_state("flash_color"), self.leds),
        #         WaitCommand(0.5),
        #         runOnce(lambda: self.leds.set_state("default"), self.leds)
        #     ).ignoringDisable(True)
        # )

        # Toggle the switchable channel on the PDH when enabled/disabled.
        (button.Trigger(lambda: DriverStation.isEnabled()).onTrue(runOnce(lambda: self.util.toggle_channel(True))
                                                                  .ignoringDisable(True))
         .onFalse(runOnce(lambda: self.util.toggle_channel(True)).ignoringDisable(True)))

        # Configuration for telemetry.
        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    # def configureTriggersSmartDash(self) -> None:
        # Activate autonomous misalignment lights.
        # button.Trigger(lambda: SmartDashboard.getBoolean("Misalignment Indicator Active?", False)).whileTrue(
        #     AutoAlignmentLEDs(self.drivetrain, self.leds, self.m_auto_start_location)
        #     .ignoringDisable(True)
        # )

        # button.Trigger(lambda: SmartDashboard.getBoolean("Logging Enabled?", False)).onTrue(
        #     SequentialCommandGroup(
        #         runOnce(lambda: DataLogManager.start()),
        #         runOnce(lambda: DriverStation.startDataLog(DataLogManager.getLog(), True)),
        #         runOnce(lambda: SignalLogger.start()),
        #         runOnce(lambda: self.alert_logging_enabled.set(True))
        #     )
        # ).onFalse(
        #     SequentialCommandGroup(
        #         runOnce(lambda: DataLogManager.stop()),
        #         runOnce(lambda: SignalLogger.stop()),
        #         runOnce(lambda: self.alert_logging_enabled.set(False))
        #     )
        # )
    def get_autonomous_command(self) -> Command:
        """Use this to pass the autonomous command to the main Robot class.
        Returns the command to run in autonomous
        """
        return self.m_chooser.getSelected()
        # if self.m_chooser.getSelected() == "DoNothing":
        #     return None
        # elif self.m_chooser.getSelected() == "BuildPlay":
        #     try:
        #         selected_auto = PathPlannerAuto(self.m_auto_start_location.getSelected() + "_Score" +
        #                                         self.m_auto_num_gp.getSelected())
        #     except FileNotFoundError:
        #         selected_auto = None
        #     return selected_auto
        # else:
        #     selected_auto = None
        #     for y in self.auto_names:
        #         if self.m_chooser.getSelected() == y:
        #             try:
        #                 selected_auto = PathPlannerAuto(y)
        #             except FileNotFoundError:
        #                 selected_auto = None
        #     return selected_auto

    def configure_test_bindings(self) -> None:
        self.configure_sys_id()

        # Point all modules in a direction
        self.driver_controller.start().and_(lambda: self.test_bindings).whileTrue(self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-1 * self.driver_controller.getLeftY()
                               -1 * self.driver_controller.getLeftX()))))

        self.driver_controller.back().and_(lambda: self.test_bindings).onTrue(
            WheelRadiusCalculator(self.drivetrain, self.timer)
        )

    def configure_sys_id(self) -> None:
        (self.driver_controller.y().and_(lambda: self.test_bindings).and_(self.driver_controller.rightTrigger())
         .whileTrue(self.drivetrain.sys_id_translation_quasistatic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.b().and_(lambda: self.test_bindings).and_(self.driver_controller.rightTrigger())
         .whileTrue(self.drivetrain.sys_id_translation_quasistatic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.a().and_(lambda: self.test_bindings).and_(self.driver_controller.rightTrigger())
         .whileTrue(self.drivetrain.sys_id_translation_dynamic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.x().and_(lambda: self.test_bindings).and_(self.driver_controller.rightTrigger())
         .whileTrue(self.drivetrain.sys_id_translation_dynamic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.y().and_(lambda: self.test_bindings).and_(self.driver_controller.rightBumper())
         .whileTrue(self.drivetrain.sys_id_rotation_quasistatic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.b().and_(lambda: self.test_bindings).and_(self.driver_controller.rightBumper())
         .whileTrue(self.drivetrain.sys_id_rotation_quasistatic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.a().and_(lambda: self.test_bindings).and_(self.driver_controller.rightBumper())
         .whileTrue(self.drivetrain.sys_id_rotation_dynamic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.x().and_(lambda: self.test_bindings).and_(self.driver_controller.rightBumper())
         .whileTrue(self.drivetrain.sys_id_rotation_dynamic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.y().and_(lambda: self.test_bindings).and_(self.driver_controller.leftBumper())
         .whileTrue(self.drivetrain.sys_id_steer_quasistatic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.b().and_(lambda: self.test_bindings).and_(self.driver_controller.leftBumper())
         .whileTrue(self.drivetrain.sys_id_steer_quasistatic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.a().and_(lambda: self.test_bindings).and_(self.driver_controller.leftBumper())
         .whileTrue(self.drivetrain.sys_id_steer_dynamic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.x().and_(lambda: self.test_bindings).and_(self.driver_controller.leftBumper())
         .whileTrue(self.drivetrain.sys_id_steer_dynamic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.y().and_(lambda: self.test_bindings).and_(self.driver_controller.leftTrigger())
         .whileTrue(self.elevator_and_arm.sys_id_quasistatic_elevator(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.b().and_(lambda: self.test_bindings).and_(self.driver_controller.leftTrigger())
         .whileTrue(self.elevator_and_arm.sys_id_quasistatic_elevator(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.a().and_(lambda: self.test_bindings).and_(self.driver_controller.leftTrigger())
         .whileTrue(self.elevator_and_arm.sys_id_dynamic_elevator(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.x().and_(lambda: self.test_bindings).and_(self.driver_controller.leftTrigger())
         .whileTrue(self.elevator_and_arm.sys_id_quasistatic_elevator(sysid.SysIdRoutine.Direction.kReverse)))

    def enable_test_bindings(self, enabled: bool) -> None:
        self.test_bindings = enabled

    def registerCommands(self):
        NamedCommands.registerCommand("rainbow_leds", runOnce(lambda: self.leds.set_state("rainbow"),
                                                              self.leds))
        NamedCommands.registerCommand("flash_green",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.leds.set_flash_color_color([255, 0, 0]),
                                                  self.leds),
                                          runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("flash_red",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.leds.set_flash_color_color([0, 255, 0]),
                                                  self.leds),
                                          runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("flash_blue",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.leds.set_flash_color_color([0, 0, 255]),
                                                  self.leds),
                                          runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("flash_purple",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.leds.set_flash_color_color([50, 149, 168]),
                                                  self.leds),
                                          runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("flash_yellow",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.leds.set_flash_color_color([255, 255, 0]),
                                                  self.leds),
                                          runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("default_leds", runOnce(lambda: self.leds.set_state("default"),
                                                              self.leds))
        NamedCommands.registerCommand("baseline", Baseline(self.drivetrain, self.timer))
        NamedCommands.registerCommand("check_drivetrain", CheckDrivetrain(self.drivetrain, self.timer))
        NamedCommands.registerCommand("override_heading_goal",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.drivetrain.set_lookahead(True)),
                                          runOnce(lambda: self.drivetrain.set_pathplanner_rotation_override("goal"))
                                        )
                                      )
        NamedCommands.registerCommand("override_heading_gp",
                                      runOnce(lambda: self.drivetrain.set_pathplanner_rotation_override("gp")))
        NamedCommands.registerCommand("disable_override_heading",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.drivetrain.set_lookahead(False)),
                                          runOnce(lambda: self.drivetrain.set_pathplanner_rotation_override("none"))
                                      ))
        NamedCommands.registerCommand("L4_left",
                                      AutoSetElevatorAndArm("L4", "left", self.elevator_and_arm))
        NamedCommands.registerCommand("L4_right",
                                      AutoSetElevatorAndArm("L4", "right", self.elevator_and_arm))
        NamedCommands.registerCommand("stow",
                                      AutoSetElevatorAndArm("stow", "stow", self.elevator_and_arm))
        NamedCommands.registerCommand("score", Score(self.elevator_and_arm, self.timer))
        NamedCommands.registerCommand("collect", Collect(self.elevator_and_arm))
