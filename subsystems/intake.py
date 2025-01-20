import phoenix6.utils
from commands2 import Subsystem
from constants import IntakeConstants

from phoenix6.hardware import TalonFX
from phoenix6.controls import MotionMagicVoltage, VoltageOut
from phoenix6.configs import TalonFXConfiguration
from phoenix6.status_code import StatusCode
from phoenix6.signals import InvertedValue
from phoenix6.utils import get_current_time_seconds, is_simulation

from rev import SparkMax, SparkBaseConfig, SparkBase

from wpilib import Mechanism2d, Color8Bit, Color, SmartDashboard, DigitalInput
from wpilib.simulation import SingleJointedArmSim
from wpimath.system.plant import DCMotor
from wpimath.units import inchesToMeters, lbsToKilograms, radiansToRotations

from math import pi, degrees


class Intake(Subsystem):
    def __init__(self):
        super().__init__()
        self._last_sim_time = get_current_time_seconds()
        self.state_values = IntakeConstants.intake_state_values
        self.state = "stow"

        self.intake_arm = TalonFX(IntakeConstants.arm_can_id, "rio")
        self.intake_arm.set_position(0)

        self.intake_arm_mm = MotionMagicVoltage(0, enable_foc=False)
        self.intake_arm_configs = TalonFXConfiguration()

        self.intake_arm_configs.current_limits.supply_current_limit = 60
        self.intake_arm_configs.current_limits.supply_current_limit_enable = True
        self.intake_arm_configs.current_limits.stator_current_limit_enable = False
        self.intake_arm_gear_ratio = IntakeConstants.gearbox_ratio
        self.intake_arm_configs.feedback.sensor_to_mechanism_ratio = self.intake_arm_gear_ratio
        if not phoenix6.utils.is_simulation():
            self.intake_arm_configs.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        else:
            self.intake_arm_configs.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        self.intake_arm_mm_configs = self.intake_arm_configs.motion_magic
        self.intake_arm_mm_configs.motion_magic_cruise_velocity = IntakeConstants.mm_cruise_velocity
        self.intake_arm_mm_configs.motion_magic_acceleration = IntakeConstants.mm_acceleration
        self.intake_arm_mm_configs.motion_magic_jerk = IntakeConstants.mm_jerk

        self.intake_arm_slot0_configs = self.intake_arm_configs.slot0
        self.intake_arm_slot0_configs.k_g = IntakeConstants.kg
        self.intake_arm_slot0_configs.k_s = IntakeConstants.ks
        self.intake_arm_slot0_configs.k_v = IntakeConstants.kv
        self.intake_arm_slot0_configs.k_a = IntakeConstants.ka
        self.intake_arm_slot0_configs.k_p = IntakeConstants.kp
        self.intake_arm_slot0_configs.k_i = IntakeConstants.ki
        self.intake_arm_slot0_configs.k_d = IntakeConstants.kd

        self.intake = SparkMax(IntakeConstants.wheel_can_id, SparkMax.MotorType.kBrushless)
        intake_config = SparkBaseConfig().smartCurrentLimit(40, 60).inverted(True)
        self.intake.configure(intake_config, SparkMax.ResetMode.kResetSafeParameters,
                              SparkMax.PersistMode.kPersistParameters)

        self.intake_speed_values = IntakeConstants.wheel_speed_values
        self.intake.setVoltage(self.intake_speed_values[self.state])

        self.gp_sensor = DigitalInput(2)

        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.intake_arm.configurator.apply(self.intake_arm_configs)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        self.intake_arm_m2d = Mechanism2d(3, 2)
        self.intake_arm_m2d_root = self.intake_arm_m2d.getRoot("arm", 1.5, 0.25)
        self.intake_arm_m2d_elbow = self.intake_arm_m2d_root.appendLigament("elbow", 1.5, 0, 6,
                                                              Color8Bit(Color.kRed))

        self.intake_arm_sim = self.intake_arm.sim_state
        self.arm_sim = SingleJointedArmSim(
            DCMotor.krakenX60(1),
            self.intake_arm_gear_ratio,
            SingleJointedArmSim.estimateMOI(inchesToMeters(20), lbsToKilograms(10)),
            inchesToMeters(20),
            -0.1,
            pi + 0.1,
            True,
            1
        )

        SmartDashboard.putNumberArray("Intake Arm Location", [inchesToMeters(5.5), inchesToMeters(0), inchesToMeters(11.5),
                                                       0, 0, 0])

        self.intake_arm_volts = VoltageOut(0, False)

        self.last_time = get_current_time_seconds()

    def set_state(self, state: str) -> None:
        self.state = state
        self.intake_arm.set_control(self.intake_arm_mm.with_position(self.state_values[state]).with_slot(0))
        if self.state == "score_coral":
            if self.get_at_target():
                self.intake.setVoltage(self.intake_speed_values[state])
        else:
            self.intake.setVoltage(self.intake_speed_values[state])


    def get_state(self) -> str:
        return self.state

    def get_sensor_on(self) -> bool:
        return not self.gp_sensor.get()

    def get_position(self) -> float:
        return self.intake_arm.get_position(True).value_as_double

    def get_at_target(self) -> bool:
        if self.state_values[self.state] - 0.1 < self.get_position() <= self.state_values[self.state] + 0.1:
            return True
        else:
            return False

    def set_voltage_direct(self, output: float):
        self.intake_arm.set_control(self.intake_arm_volts.with_output(output)
                               .with_limit_forward_motion(self.get_forward_limit_triggered())
                               .with_limit_reverse_motion(self.get_reverse_limit_triggered()))

    def get_forward_limit_triggered(self) -> bool:
        if self.intake_arm.get_position().value_as_double > 0.5:
            return True
        else:
            return False

    def get_reverse_limit_triggered(self) -> bool:
        if self.intake_arm.get_position().value_as_double < 0:
            return True
        else:
            return False

    def update_sim(self):
        current_time = get_current_time_seconds()
        dt = current_time - self.last_time
        self.last_time = current_time
        self.arm_sim.setInput(0, self.intake_arm_sim.motor_voltage)
        self.arm_sim.update(dt)
        self.intake_arm_sim.set_raw_rotor_position(radiansToRotations(self.arm_sim.getAngle() * self.intake_arm_gear_ratio))
        self.intake_arm_sim.set_rotor_velocity(radiansToRotations(self.arm_sim.getVelocity() * self.intake_arm_gear_ratio))

    def periodic(self) -> None:
        if is_simulation():
            self.update_sim()
            self.intake_arm_m2d_elbow.setAngle(degrees(self.arm_sim.getAngle()))
            SmartDashboard.putNumberArray("Intake Arm Location", [inchesToMeters(5.5), inchesToMeters(0), inchesToMeters(11.5),
                                                           0, 0, self.arm_sim.getAngle()])
        else:
            self.intake_arm_m2d_elbow.setAngle(self.intake_arm.get_position().value_as_double)

        #if self.state == "score_coral":
        #    if self.get_at_target():
        #        self.intake.setVoltage(-2.4)
        #elif self.state == "intake_coral":
        #    self.intake.setVoltage(12)
        #else:
        #    self.intake.setVoltage(0)

        SmartDashboard.putData("Arm M2D", self.intake_arm_m2d)
        SmartDashboard.putNumber("Intake Position", self.intake_arm.get_position().value_as_double)