from commands2 import Subsystem, Command, SequentialCommandGroup, WaitCommand
from commands2.cmd import run, runOnce
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import MotionMagicVoltage, VoltageOut, Follower, PositionVoltage
from phoenix6.configs import TalonFXConfiguration
from phoenix6.status_code import StatusCode
from phoenix6.utils import get_current_time_seconds, is_simulation
from phoenix6.signals import StaticFeedforwardSignValue
from phoenix6 import SignalLogger

from wpilib import Mechanism2d, Color8Bit, Color, SmartDashboard, DigitalInput, PWMSparkMax
from wpilib.simulation import ElevatorSim, SingleJointedArmSim
from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations

from math import pi, degrees

from constants import ElevatorConstants
from constants import ArmConstants


class ElevatorAndArmSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self._last_sim_time = get_current_time_seconds()

        self.intake = PWMSparkMax(1)
        self.intake.set(0)

        self.elevator_state_values = ElevatorConstants.state_values
        self.arm_state_values = ArmConstants.state_values
        self.elevator_state = "stow"
        self.arm_state = "stow"
        self.last_time = get_current_time_seconds()
        self.setName("ElevatorAndArm")
        self.accel_limit_scalar = 1

        self.lift_main_mm = MotionMagicVoltage(0, enable_foc=False)
        self.lift_main_vo = VoltageOut(0, enable_foc=False)
        self.lift_main_pid = PositionVoltage(0, enable_foc=False)

        self.wrist_mm = MotionMagicVoltage(0, enable_foc=False)
        self.wrist_vo = VoltageOut(0, enable_foc=False)
        self.wrist_pid = PositionVoltage(0, enable_foc=False)

        for motor_id in ElevatorConstants.can_ids:
            motor = TalonFX(motor_id, "rio")
            motor_config = TalonFXConfiguration()

            motor.set_position(0)

            motor_config.current_limits.supply_current_limit = ElevatorConstants.supply_current_limit
            motor_config.current_limits.supply_current_limit_enable = ElevatorConstants.use_supply_current_limit
            motor_config.current_limits.stator_current_limit_enable = False
            motor_config.feedback.sensor_to_mechanism_ratio = ElevatorConstants.gearbox_ratio

            motor_mm_config = motor_config.motion_magic
            motor_mm_config.motion_magic_cruise_velocity = ElevatorConstants.mm_cruise_velocity
            motor_mm_config.motion_magic_acceleration = ElevatorConstants.mm_acceleration
            motor_mm_config.motion_magic_jerk = ElevatorConstants.mm_jerk

            motor_slot0_config = motor_config.slot0
            motor_slot0_config.k_g = ElevatorConstants.kg
            motor_slot0_config.k_s = ElevatorConstants.ks
            motor_slot0_config.k_v = ElevatorConstants.kv
            motor_slot0_config.k_a = ElevatorConstants.ka
            motor_slot0_config.k_p = ElevatorConstants.kp
            motor_slot0_config.k_i = ElevatorConstants.ki
            motor_slot0_config.k_d = ElevatorConstants.kd

            status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
            for _ in range(0, 5):
                status = motor.configurator.apply(motor_config)
                if status.is_ok():
                    print("Configuration applied.")
                    break
            if not status.is_ok():
                print(f"Could not apply configs, error code: {status.name}")

            if motor_id != ElevatorConstants.can_ids[0]:
                follower_control_request = Follower(ElevatorConstants.can_ids[0], True)
                motor.set_control(follower_control_request)
            else:
                self.lift_main = motor

        self.lift_sim = self.lift_main.sim_state
        self.elevator_sim = ElevatorSim(
            gearbox=DCMotor.krakenX60(len(ElevatorConstants.can_ids)),
            gearing=ElevatorConstants.gearbox_ratio,
            carriageMass=ElevatorConstants.carriage_weight,
            drumRadius=ElevatorConstants.drum_diameter_m / 2,
            minHeight=ElevatorConstants.min_height_m,
            maxHeight=ElevatorConstants.max_height_m,
            simulateGravity=True,
            startingHeight=0.0,
            measurementStdDevs=[0.001, 0.001],
        )

        self.sys_id_routine_elevator = SysIdRoutine(
            SysIdRoutine.Config(
                stepVoltage=4.0,
                recordState=lambda state: SignalLogger.write_string("state", SysIdRoutineLog.stateEnumToString(state))
            ),
            SysIdRoutine.Mechanism(
                lambda volts: self.lift_main.set_control(self.lift_main_vo.with_output(volts)),
                lambda log: None,
                self
            )
        )

        self.wrist = TalonFX(ArmConstants.wrist_can_id, "rio")
        wrist_config = TalonFXConfiguration()

        wrist_config.current_limits.supply_current_limit = ArmConstants.supply_current_limit
        wrist_config.current_limits.supply_current_limit_enable = ArmConstants.use_supply_current_limit
        wrist_config.current_limits.stator_current_limit_enable = False
        wrist_config.feedback.sensor_to_mechanism_ratio = ArmConstants.gearbox_ratio
        self.wrist.set_position(0)

        wrist_mm_config = wrist_config.motion_magic
        wrist_mm_config.motion_magic_cruise_velocity = ArmConstants.mm_cruise_velocity
        wrist_mm_config.motion_magic_acceleration = ArmConstants.mm_acceleration
        wrist_mm_config.motion_magic_jerk = ArmConstants.mm_jerk

        wrist_slot0_config = wrist_config.slot0
        wrist_slot0_config.k_g = ArmConstants.kg
        wrist_slot0_config.k_s = ElevatorConstants.ks
        wrist_slot0_config.k_v = ElevatorConstants.kv
        wrist_slot0_config.k_a = ElevatorConstants.ka
        wrist_slot0_config.k_p = ElevatorConstants.kp
        wrist_slot0_config.k_i = ElevatorConstants.ki
        wrist_slot0_config.k_d = ElevatorConstants.kd

        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.wrist.configurator.apply(wrist_config)
            if status.is_ok():
                print("Configuration applied.")
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        self.wrist_sim = self.wrist.sim_state
        self.arm_sim = SingleJointedArmSim(
            DCMotor.krakenX60(1),
            ArmConstants.gearbox_ratio,
            SingleJointedArmSim.estimateMOI(ArmConstants.arm_length, ArmConstants.arm_weight),
            ArmConstants.arm_length,
            -0.1,
            pi + 0.1,
            True,
            1
        )

        self.sys_id_routine_arm = SysIdRoutine(
            SysIdRoutine.Config(
                stepVoltage=4.0,
                recordState=lambda state: SignalLogger.write_string("state", SysIdRoutineLog.stateEnumToString(state))
            ),
            SysIdRoutine.Mechanism(
                lambda volts: self.wrist.set_control(self.wrist_vo.with_output(volts)),
                lambda log: None,
                self
            )
        )

        self.lift_m2d = Mechanism2d(25, ElevatorConstants.max_height_in)
        lift_root = self.lift_m2d.getRoot("Lift Root", 10, 0)
        self.elevator_m2d = lift_root.appendLigament("Elevator",
                                                     self.elevator_sim.getPositionInches(),
                                                     ElevatorConstants.elevator_angle_degrees,
                                                     6,
                                                     Color8Bit(Color.kRed))
        self.arm_root = self.lift_m2d.getRoot("Arm Root", 10, 0)
        self.arm_m2d = self.arm_root.appendLigament("Arm",
                                                    5,
                                                    90,
                                                    10,
                                                    Color8Bit(Color.kBlue))

        self.set_arm_state("stow")

    def set_elevator_state(self, state: str) -> None:
        self.elevator_state = state
        self.lift_main.set_control(self.lift_main_mm.with_position(self.elevator_state_values[state]).with_slot(0))

    def set_arm_state(self, state: str) -> None:
        self.arm_state = state
        self.wrist.set_control(self.wrist_mm.with_position(self.arm_state_values[state]).with_slot(0))

    def set_elevator_manual(self, voltage: float) -> None:
        self.elevator_state = "manual"
        self.lift_main.set_control(self.lift_main_vo.with_output(voltage)
                                   .with_limit_reverse_motion(self.get_reverse_limit_triggered())
                                   .with_limit_forward_motion(self.get_forward_limit_triggered()))

    def set_arm_manual(self, voltage: float) -> None:
        self.arm_state = "manual"
        self.wrist.set_control(self.wrist_vo.with_output(voltage))

    def set_elevator_manual_off(self) -> None:
        self.elevator_state = "manual_off"
        self.lift_main.set_control(self.lift_main_pid.with_position(self.get_elevator_position())
                                   .with_limit_reverse_motion(self.get_reverse_limit_triggered())
                                   .with_limit_forward_motion(self.get_forward_limit_triggered()))

    def set_arm_manual_off(self) -> None:
        self.arm_state = "manual_off"
        self.wrist.set_control(self.wrist_pid.with_position(self.get_arm_position()))

    def get_elevator_state(self) -> str:
        return self.elevator_state

    def get_arm_state(self) -> str:
        return self.arm_state

    def get_elevator_position(self) -> float:
        return self.lift_main.get_position(True).value_as_double

    def get_arm_position(self) -> float:
        return self.wrist.get_position(True).value_as_double

    def get_elevator_at_target(self) -> bool:
        if (self.elevator_state_values[self.elevator_state] - ElevatorConstants.elevator_at_target_threshold < self.get_elevator_position() <=
                self.elevator_state_values[self.elevator_state] + ElevatorConstants.elevator_at_target_threshold):
            return True
        else:
            return False

    def get_arm_at_target(self) -> bool:
        if (self.arm_state_values[self.arm_state] - ArmConstants.arm_at_target_threshold < self.get_arm_position() <=
                self.arm_state_values[self.arm_state] + ArmConstants.arm_at_target_threshold):
            return True
        else:
            return False

    def get_forward_limit_triggered(self) -> bool:
        if self.lift_main.get_position().value_as_double > ElevatorConstants.elevator_upper_limit:
            return True
        else:
            return False

    def get_reverse_limit_triggered(self) -> bool:
        if self.lift_main.get_position().value_as_double < ElevatorConstants.elevator_lower_limit:
            return True
        else:
            return False

    def sys_id_quasistatic_elevator(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_elevator.quasistatic(direction)

    def sys_id_dynamic_elevator(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_elevator.dynamic(direction)

    def sys_id_quasistatic_arm(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_arm.quasistatic(direction)

    def sys_id_dynamic_arm(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_arm.dynamic(direction)

    def set_accel_limit(self, scalar: float) -> None:
        self.accel_limit_scalar = scalar

    def get_accel_limit(self) -> float:
        return self.accel_limit_scalar

    def update_sim(self) -> None:
        current_time = get_current_time_seconds()
        dt = current_time - self.last_time
        self.last_time = current_time

        self.elevator_sim.setInput(0, self.lift_sim.motor_voltage)
        self.elevator_sim.update(dt)
        self.lift_sim.set_raw_rotor_position(self.elevator_sim.getPositionInches() * ElevatorConstants.gearbox_ratio /
                                             (ElevatorConstants.drum_diameter_in * pi))
        self.lift_sim.set_rotor_velocity(self.elevator_sim.getVelocity() * ElevatorConstants.gearbox_ratio)

        self.arm_sim.setInput(0, self.wrist_sim.motor_voltage)
        self.arm_sim.update(dt)
        self.wrist_sim.set_raw_rotor_position(radiansToRotations(self.arm_sim.getAngle() * ArmConstants.gearbox_ratio))
        self.wrist_sim.set_rotor_velocity(radiansToRotations(self.arm_sim.getVelocity() * ArmConstants.gearbox_ratio))

    def periodic(self) -> None:
        if is_simulation():
            self.update_sim()
            self.elevator_m2d.setLength(self.elevator_sim.getPositionInches())
            self.arm_root.setPosition(10, self.elevator_m2d.getLength())
            self.arm_m2d.setAngle(degrees(self.arm_sim.getAngle()))
            SmartDashboard.putData("Elevator M2D", self.lift_m2d)
            SmartDashboard.putString("Elevator Position", str(self.elevator_sim.getPositionInches()))
            SmartDashboard.putString("Arm Setpoint", self.get_arm_state())
        # else:
        #     self.elevator_m2d.setLength(self.lift_main.get_position().value_as_double * pi *
        #                                 ElevatorConstants.drum_diameter_in)
        SmartDashboard.putString("Lift Motor Position", str(self.lift_main.get_position()))
        SmartDashboard.putString("Elevator Setpoint", self.elevator_state)
