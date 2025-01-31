import phoenix6.utils
from commands2 import Subsystem
from constants import ClimberConstants

from phoenix6.hardware import TalonFX
from phoenix6.controls import VoltageOut
from phoenix6.configs import TalonFXConfiguration
from phoenix6.status_code import StatusCode
from phoenix6.signals import InvertedValue
from phoenix6.utils import get_current_time_seconds, is_simulation

from wpilib import Mechanism2d, Color8Bit, Color, SmartDashboard, DigitalInput
from wpilib.simulation import SingleJointedArmSim
from wpimath.system.plant import DCMotor
from wpimath.units import inchesToMeters, lbsToKilograms, radiansToRotations

from math import pi, degrees


class Climber(Subsystem):
    def __init__(self):
        super().__init__()
        self._last_sim_time = get_current_time_seconds()

        self.climber_arm = TalonFX(ClimberConstants.climber_can_id, "rio")
        self.climber_arm.set_position(0)

        self.climber_arm_configs = TalonFXConfiguration()

        self.climber_arm_configs.current_limits.supply_current_limit = 60
        self.climber_arm_configs.current_limits.supply_current_limit_enable = True
        self.climber_arm_configs.current_limits.stator_current_limit_enable = False
        self.climber_arm_gear_ratio = ClimberConstants.gearbox_ratio
        self.climber_arm_configs.feedback.sensor_to_mechanism_ratio = self.climber_arm_gear_ratio
        if not phoenix6.utils.is_simulation():
            self.climber_arm_configs.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        else:
            self.climber_arm_configs.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.climber_arm.configurator.apply(self.climber_arm_configs)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        self.climber_arm_m2d = Mechanism2d(3, 2)
        self.climber_arm_m2d_root = self.climber_arm_m2d.getRoot("arm", 1.5, 0.25)
        self.climber_arm_m2d_elbow = self.climber_arm_m2d_root.appendLigament("elbow", 1.5, 0, 6,
                                                              Color8Bit(Color.kRed))

        self.climber_arm_sim = self.climber_arm.sim_state
        self.arm_sim = SingleJointedArmSim(
            DCMotor.krakenX60FOC(1),
            self.climber_arm_gear_ratio,
            SingleJointedArmSim.estimateMOI(inchesToMeters(20), lbsToKilograms(10)),
            inchesToMeters(20),
            -0.1,
            pi + 0.1,
            True,
            1
        )

        self.climber_arm_volts = VoltageOut(0, True)

        self.debug_mode = False

        self.last_time = get_current_time_seconds()

    def get_position(self) -> float:
        return self.climber_arm.get_position(True).value_as_double

    def set_voltage_direct(self, output: float):
        self.climber_arm.set_control(self.climber_arm_volts.with_output(output)
                                     .with_limit_forward_motion(self.get_forward_limit_triggered())
                                     .with_limit_reverse_motion(self.get_reverse_limit_triggered()))

    def get_forward_limit_triggered(self) -> bool:
        if self.climber_arm.get_position().value_as_double > 0.5:
            return True
        else:
            return False

    def get_reverse_limit_triggered(self) -> bool:
        if self.climber_arm.get_position().value_as_double < 0:
            return True
        else:
            return False

    def set_climber_manual(self, voltage: float) -> None:
        self.climber_arm.set_control(self.climber_arm_volts.with_output(voltage)
                                     .with_limit_reverse_motion(self.get_reverse_limit_triggered())
                                     .with_limit_forward_motion(self.get_forward_limit_triggered()))

    def set_debug_mode(self) -> None:
        self.debug_mode = not self.debug_mode

    def update_sim(self):
        current_time = get_current_time_seconds()
        dt = current_time - self.last_time
        self.last_time = current_time
        self.arm_sim.setInput(0, self.climber_arm_sim.motor_voltage)
        self.arm_sim.update(dt)
        self.climber_arm_sim.set_raw_rotor_position(radiansToRotations(self.arm_sim.getAngle() * self.climber_arm_gear_ratio))
        self.climber_arm_sim.set_rotor_velocity(radiansToRotations(self.arm_sim.getVelocity() * self.climber_arm_gear_ratio))

    def periodic(self) -> None:
        if is_simulation():
            self.update_sim()
            self.climber_arm_m2d_elbow.setAngle(degrees(self.arm_sim.getAngle()))
            SmartDashboard.putData("Climber Arm M2D", self.climber_arm_m2d)
        # else:
        #     self.climber_arm_m2d_elbow.setAngle(self.climber_arm.get_position().value_as_double)

        if self.debug_mode:
            SmartDashboard.putNumber("Climber Position", self.climber_arm.get_position().value_as_double)