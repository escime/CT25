from commands2 import Subsystem
from wpilib import AddressableLED, Timer, SmartDashboard, Color
from constants import LEDConstants
from random import randint


class LEDs(Subsystem):
    def __init__(self, timer: Timer):
        super().__init__()
        self.timer = timer
        self.chain = AddressableLED(LEDConstants.port)
        self.chain.setLength(LEDConstants.strip_length)
        self.buffer = [AddressableLED.LEDData(0, 0, 0)] * LEDConstants.strip_length
        self.chain.setData(self.buffer)
        self.chain.start()
        self.state = "default"

        # Prepare default pattern
        self.default_pattern = [AddressableLED.LEDData(149, 50, 168)] * 2
        for i in range(0, 2):
            self.default_pattern.append(AddressableLED.LEDData(int(149 * 0.75), int(50 * 0.75), int(168 * 0.75)))
        for i in range(0, 3):
            self.default_pattern.append(AddressableLED.LEDData(int(149 * 0.5), int(50 * 0.5), int(168 * 0.5)))
        for i in range(0, 7):
            self.default_pattern.append(AddressableLED.LEDData(int(149 * 0.25), int(50 * 0.25), int(168 * 0.25)))
        for i in range(0, 7):
            self.default_pattern.append(AddressableLED.LEDData(int(149 * 0.1), int(50 * 0.1), int(168 * 0.1)))
        for i in range(0, LEDConstants.strip_length - 21):
            self.default_pattern.append(AddressableLED.LEDData(0, 0, 0))

        # Prepare no-animation light.
        self.no_pattern = [AddressableLED.LEDData(149, 50, 168)] * LEDConstants.strip_length
        self.gp_held_simple = [AddressableLED.LEDData(0, 255, 0)] * LEDConstants.strip_length

        # Prepare gp_held pattern
        self.gp_held_pattern = [AddressableLED.LEDData(149, 50, 168)] * 2
        for i in range(0, 2):
            self.gp_held_pattern.append(AddressableLED.LEDData(int(149 * 0.75), int(50 * 0.75), int(168 * 0.75)))
        for i in range(0, 3):
            self.gp_held_pattern.append(AddressableLED.LEDData(int(149 * 0.5), int(100 * 0.75), int(168 * 0.5)))
        for i in range(0, 7):
            self.gp_held_pattern.append(AddressableLED.LEDData(int(149 * 0.25), int(200 * 0.5), int(168 * 0.25)))
        for i in range(0, 7):
            self.gp_held_pattern.append(AddressableLED.LEDData(int(149 * 0.1), int(250 * 0.75), int(168 * 0.1)))
        for i in range(0, LEDConstants.strip_length - 21):
            self.gp_held_pattern.append(AddressableLED.LEDData(0, 255, 0))

        # Prepare rainbow pattern
        self.rainbow_pattern = []
        multiplier = 2
        for i in range(0, int(LEDConstants.strip_length)):
            for j in range(0, multiplier):
                self.rainbow_pattern.append(AddressableLED.LEDData(255, 0, 0))
            for j in range(0, multiplier):
                self.rainbow_pattern.append(AddressableLED.LEDData(255, 72, 0))
            for j in range(0, multiplier):
                self.rainbow_pattern.append(AddressableLED.LEDData(255, 213, 0))
            for j in range(0, multiplier):
                self.rainbow_pattern.append(AddressableLED.LEDData(153, 255, 0))
            for j in range(0, multiplier):
                self.rainbow_pattern.append(AddressableLED.LEDData(0, 255, 0))
            for j in range(0, multiplier):
                self.rainbow_pattern.append(AddressableLED.LEDData(0, 47, 255))
            for j in range(0, multiplier):
                self.rainbow_pattern.append(AddressableLED.LEDData(128, 0, 255))
            for j in range(0, multiplier):
                self.rainbow_pattern.append(AddressableLED.LEDData(255, 0, 255))
        self.rainbow_pattern = self.rainbow_pattern[0:LEDConstants.strip_length]

        # Prepare flash color settings
        self.flash_color_rate = 3
        self.flash_color_color = [0, 255, 0]
        self.flash_color_state = True

        # Prepare shoot settings
        self.shoot_pattern = [AddressableLED.LEDData(0, 0, 255)] * 5
        for i in range(0, LEDConstants.strip_length - 5):
            self.shoot_pattern.append(AddressableLED.LEDData(0, 0, 0))

        # Prepare timer lights settings
        self.timer_lights_on = True
        self.timer_lights_time = 10
        self.timer_lights_segment = int(LEDConstants.strip_length / self.timer_lights_time)
        self.timer_lights_pattern = [AddressableLED.LEDData(255, 0, 0)] * \
            (LEDConstants.strip_length - self.timer_lights_segment)
        for i in range(0, self.timer_lights_segment):
            self.timer_lights_pattern.append(AddressableLED.LEDData(0, 0, 0))

        # Prepare alignment settings
        self.align_pattern = [AddressableLED.LEDData(255, 0, 0)] * LEDConstants.strip_length
        self.misalignment = 0

        # Prepare flame settings
        self.cooling = 10
        self.sparking = 150
        self.heat = [0] * LEDConstants.strip_length
        self.flame_color = [255, 0, 0]
        self.flame_pattern = [AddressableLED.LEDData(255, 0, 0)] * LEDConstants.strip_length

        # Set up settings for notifiers
        self.notifier_on = False
        self.priority_notifier = [255, 0, 0]

        # Setup time variable default settings
        self.counter = 1
        self.time_default_pattern = [AddressableLED.LEDData(149, 50, 168)] * 2
        for i in range(0, 2):
            self.time_default_pattern.append(AddressableLED.LEDData(int(149 * 0.75), int(50 * 0.75), int(168 * 0.75)))
        for i in range(0, 3):
            self.time_default_pattern.append(AddressableLED.LEDData(int(149 * 0.5), int(50 * 0.5), int(168 * 0.5)))
        for i in range(0, 7):
            self.time_default_pattern.append(AddressableLED.LEDData(int(149 * 0.25), int(50 * 0.25), int(168 * 0.25)))
        for i in range(0, 7):
            self.time_default_pattern.append(AddressableLED.LEDData(int(149 * 0.1), int(50 * 0.1), int(168 * 0.1)))
        for i in range(0, LEDConstants.strip_length - 21):
            self.time_default_pattern.append(AddressableLED.LEDData(0, 0, 0))

        self.display_buffer = [0] * LEDConstants.strip_length

        self.re_enter_default = True
        self.re_enter_gp_held = True

        self.last_time = self.timer.get()

    def set_state(self, target_state: str) -> None:
        """Set the current state of the subsystem."""
        self.state = target_state
        if target_state == "default":
            self.re_enter_default = True
        if target_state == "timer_lights":
            self.buffer = [AddressableLED.LEDData(255, 0, 0)] * LEDConstants.strip_length
        # if target_state == "time_variable_default":
        #     self.counter = 1
        #     self.time_default_pattern = [AddressableLED.LEDData(149, 50, 168)] * 2
        #     for i in range(0, 2):
        #         self.time_default_pattern.append(
        #             AddressableLED.LEDData(int(149 * 0.75), int(50 * 0.75), int(168 * 0.75)))
        #     for i in range(0, 3):
        #         self.time_default_pattern.append(AddressableLED.LEDData(int(149 * 0.5), int(50 * 0.5), int(168 * 0.5)))
        #     for i in range(0, 7):
        #         self.time_default_pattern.append(
        #             AddressableLED.LEDData(int(149 * 0.25), int(50 * 0.25), int(168 * 0.25)))
        #     for i in range(0, 7):
        #         self.time_default_pattern.append(AddressableLED.LEDData(int(149 * 0.1), int(50 * 0.1), int(168 * 0.1)))
        #     for i in range(0, LEDConstants.strip_length - 21):
        #         self.time_default_pattern.append(AddressableLED.LEDData(0, 0, 0))

    def periodic(self) -> None:
        if self.state != "default" and self.state != "gp_held":
            self.re_enter_gp_held = True
            self.re_enter_default = True
            if self.state == "flash_color":
                self.flash_color()
            elif self.state == "shoot":
                self.shoot()
            elif self.state == "gp_held":
                self.gp_held()
            elif self.state == "rainbow":
                self.rainbow()
            elif self.state == "timer_lights":
                self.timer_lights()
            elif self.state == "align":
                self.align()
            elif self.state == "flames":
                self.flames()
            # elif self.state == "time_variable_default":
            #     self.time_variable_default()

            if self.notifier_on:
                self.buffer = self.buffer[:-5]
                for i in range(0, 5):
                    self.buffer.append(AddressableLED.LEDData(self.priority_notifier[0], self.priority_notifier[1],
                                                              self.priority_notifier[2]))

            self.chain.setData(self.buffer)
            # for i in range(0, LEDConstants.strip_length):
            #     self.display_buffer[i] = Color(self.buffer[i].r, self.buffer[i].g, self.buffer[i].b).hexString()
            # SmartDashboard.putStringArray("LED Display", self.display_buffer)
        else:
            # self.default()
            if self.state == "default":
                if self.re_enter_default:
                    self.buffer = self.no_pattern
                    self.re_enter_default = False
                    self.chain.setData(self.buffer)
            if self.state == "gp_held":
                if self.re_enter_gp_held:
                    self.buffer = self.gp_held_simple
                    self.re_enter_gp_held = False
                    self.chain.setData(self.buffer)

    def default(self) -> None:
        """Logic for running default animation."""
        if self.timer.get() - 0.05 > self.last_time:
            self.buffer = self.default_pattern
            self.default_pattern = self.default_pattern[1:] + self.default_pattern[:1]
            self.last_time = self.timer.get()

    def time_variable_default(self) -> None:
        self.counter += 1
        if self.timer.get() - (0.05 * self.counter) > self.last_time:
            self.buffer = self.time_default_pattern
            self.time_default_pattern = self.time_default_pattern[1:] + self.time_default_pattern[:1]
            self.last_time = self.timer.get()
        if self.counter > LEDConstants.strip_length:
            self.counter = 0

    def flash_color(self) -> None:
        """Flash a specified color at a specified frequency."""
        if self.timer.get() - (1 / self.flash_color_rate) > self.last_time and self.flash_color_state:
            self.buffer = [AddressableLED.LEDData(self.flash_color_color[0], self.flash_color_color[1],
                                                  self.flash_color_color[2])] * LEDConstants.strip_length
            self.flash_color_state = False
            self.last_time = self.timer.get()
        elif self.timer.get() - (1 / self.flash_color_rate) > self.last_time and not self.flash_color_state:
            self.buffer = [AddressableLED.LEDData(0, 0, 0)] * LEDConstants.strip_length
            self.flash_color_state = True
            self.last_time = self.timer.get()

    def set_flash_color_color(self, color: []) -> None:
        """Set the color for Flash Color."""
        self.flash_color_color = color

    def set_flash_color_rate(self, rate: int) -> None:
        """Set the rate for Flash Color."""
        self.flash_color_rate = rate

    def shoot(self) -> None:
        """Run the LEDs towards the shooter and then hold in place."""
        shoot_complete = False
        if self.shoot_pattern[-1].b == 255:
            shoot_complete = True
            self.buffer = self.shoot_pattern
        if self.timer.get() - 0.02 > self.last_time and not shoot_complete:
            self.buffer = self.shoot_pattern
            self.shoot_pattern = self.shoot_pattern[-1:] + self.shoot_pattern[:-1]
            self.last_time = self.timer.get()

    def reset_shoot(self) -> None:
        """Reset the shoot animation."""
        self.shoot_pattern = [AddressableLED.LEDData(0, 0, 255)] * 5
        for i in range(0, LEDConstants.strip_length - 5):
            self.shoot_pattern.append(AddressableLED.LEDData(0, 0, 0))

    def gp_held(self) -> None:
        """Run the animation for holding a game piece."""
        if self.timer.get() - 0.05 > self.last_time:
            self.buffer = self.gp_held_pattern
            self.gp_held_pattern = self.gp_held_pattern[1:] + self.gp_held_pattern[:1]
            self.last_time = self.timer.get()

    def rainbow(self) -> None:
        """Run the rainbow shift animation."""
        if self.timer.get() - 0.02 > self.last_time:
            self.buffer = self.rainbow_pattern
            self.rainbow_pattern = self.rainbow_pattern[1:] + self.rainbow_pattern[:1]
            self.last_time = self.timer.get()

    def timer_lights(self) -> None:
        """Run a timer animation on the lights for a set time."""
        self.timer_lights_on = True
        if self.timer.get() - 1 > self.last_time and self.timer_lights_pattern[0].r == 0:
            self.timer_lights_on = False
            self.buffer = self.timer_lights_pattern
        if self.timer.get() - 1 > self.last_time and self.timer_lights_on:
            self.buffer = self.timer_lights_pattern
            self.timer_lights_pattern = self.timer_lights_pattern[self.timer_lights_segment:]
            for i in range(0, self.timer_lights_segment):
                self.timer_lights_pattern.append(AddressableLED.LEDData(0, 0, 0))
            self.last_time = self.timer.get()

    def set_timer_lights_time(self, time: float) -> None:
        """Set the time for the timer lights."""
        self.timer_lights_on = True
        self.timer_lights_time = time
        self.timer_lights_segment = int(LEDConstants.strip_length / self.timer_lights_time)
        self.timer_lights_pattern = [AddressableLED.LEDData(255, 0, 0)] * \
            (LEDConstants.strip_length - self.timer_lights_segment)
        for i in range(0, self.timer_lights_segment):
            self.timer_lights_pattern.append(AddressableLED.LEDData(0, 0, 0))

    def reset_timer_lights(self) -> None:
        self.timer_lights_on = True
        self.timer_lights_pattern = [AddressableLED.LEDData(255, 0, 0)] * \
            (LEDConstants.strip_length - self.timer_lights_segment)
        for i in range(0, self.timer_lights_segment):
            self.timer_lights_pattern.append(AddressableLED.LEDData(0, 0, 0))

    def align(self) -> None:
        alignment_amount = LEDConstants.strip_length - int(LEDConstants.strip_length -
                                                           (LEDConstants.strip_length * abs(self.misalignment)))
        if self.misalignment > 0:
            color = [255, 0, 0]
        else:
            color = [0, 0, 255]
        for i in range(0, LEDConstants.strip_length - alignment_amount):
            self.align_pattern[i] = AddressableLED.LEDData(color[0], color[1], color[2])
        for i in range(LEDConstants.strip_length - alignment_amount, LEDConstants.strip_length):
            self.align_pattern[i] = AddressableLED.LEDData(0, 0, 0)
        self.buffer = self.align_pattern

    def set_misalignment(self, target: float, current: float) -> None:
        self.misalignment = (target - current) / 180

    def flames(self) -> None:
        if self.timer.get() - 0.02 > self.last_time:
            # Cool down all cells
            for i in range(0, LEDConstants.strip_length):
                cooldown = randint(0, self.cooling)

                if cooldown > self.heat[i]:
                    self.heat[i] = 0
                else:
                    self.heat[i] = self.heat[i] - cooldown

            # Heat from each cell drips up and diffuses
            for k in range(LEDConstants.strip_length - 1, 2, -1):
                self.heat[k] = (self.heat[k - 1] + self.heat[k - 2] + self.heat[k - 2]) / 3

            # Cool LEDs at end of strip more
            for n in range(1, int(LEDConstants.strip_length / 2)):
                cool_amt = randint(0, 30)
                if self.heat[-1 * n] - cool_amt < 0:
                    self.heat[-1 * n] = 0
                else:
                    self.heat[-1 * n] = self.heat[-1 * n] - cool_amt

            # Randomly ignite new "sparks"
            if randint(0, 255) < self.sparking:
                y = randint(0, 20)
                self.heat[y] = self.heat[y] + randint(200, 255)

            for m in range(0, int(LEDConstants.strip_length / 10)):
                self.heat[m] = 255

            # Convert heat to LED colors
            for j in range(0, LEDConstants.strip_length):
                self.flame_pattern[j] = AddressableLED.LEDData(int(self.flame_color[0] * (self.heat[j] / 255)),
                                                               int(self.flame_color[1] * (self.heat[j] / 255)),
                                                               int(self.flame_color[2] * (self.heat[j] / 255)))

            self.buffer = self.flame_pattern
            self.last_time = self.timer.get()

    def reset_flames(self) -> None:
        self.flame_pattern = [AddressableLED.LEDData(0, 0, 0)] * LEDConstants.strip_length
        self.heat = [0] * LEDConstants.strip_length

    def set_notifier(self, color: [int, int, int]) -> None:
        if color[0] == -1 and color[1] == -1 and color[2] == -1:
            self.notifier_on = False
        else:
            self.notifier_on = True
            self.priority_notifier = color
