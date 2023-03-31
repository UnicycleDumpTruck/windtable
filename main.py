# SPDX-FileCopyrightText: 2021 John Furcean
# SPDX-License-Identifier: MIT

"""I2C rotary encoders in array of Knobs."""

import time
import digitalio
from analogio import AnalogIn
import board
from adafruit_seesaw import seesaw, rotaryio
from adafruit_seesaw import digitalio as sdio
import neopixel
import digitalio as dio
from adafruit_debouncer import Debouncer
# from microcontroller import watchdog as w
# from watchdog import WatchDogMode

import pwmio
from adafruit_motor import servo

RESOLUTION = 0  # Only change on every nth position.
INCREMENT_PER_ENCODER_TICK = 2
UPPER_AZIMUTH_LIMIT = 135
LOWER_AZIMUTH_LIMIT = 30
WIND_VOLTAGE_THRESHOLD = 0.5

# create a PWMOut object on Pin A15.
pwm = pwmio.PWMOut(board.A15, duty_cycle=2 ** 15, frequency=50)
fan_speed_pwm = pwmio.PWMOut(board.A12, duty_cycle=2**15, frequency=50)

fan_servo = servo.Servo(pwm)
fan_speed = servo.Servo(fan_speed_pwm)
fan_speed.angle = 0 # Start with fan off. Speed controller needs to see change after powerup.


# Input from Start button
fan_button_pin = digitalio.DigitalInOut(board.A14)
fan_button_pin.direction = dio.Direction.INPUT
fan_button_pin.pull = dio.Pull.UP
fan_button = Debouncer(fan_button_pin)

fan_led_pin = dio.DigitalInOut(board.D17)
fan_led_pin.direction = dio.Direction.OUTPUT
fan_led_pin.drive_mode = dio.DriveMode.PUSH_PULL
fan_led_pin.value = True


# Inputs from Windmill motors
windmill_a_analog_in = AnalogIn(board.A0)
windmill_b_analog_in = AnalogIn(board.A1)
windmill_c_analog_in = AnalogIn(board.A2)

# Outputs to lighting transistorsFalse
# Highrises
scene_a_pin = dio.DigitalInOut(board.D5)
scene_a_pin.direction = dio.Direction.OUTPUT
scene_a_pin.drive_mode = dio.DriveMode.PUSH_PULL
scene_a_pin.value = False

# Houses
scene_b_pin = dio.DigitalInOut(board.D6)
scene_b_pin.direction = dio.Direction.OUTPUT
scene_b_pin.drive_mode = dio.DriveMode.PUSH_PULL
scene_b_pin.value = False

# Train
scene_c_pin = dio.DigitalInOut(board.D7)
scene_c_pin.direction = dio.Direction.OUTPUT
scene_c_pin.drive_mode = dio.DriveMode.PUSH_PULL
scene_c_pin.value = False


# w.timeout=8 # Set a timeout of 4 seconds
# w.mode = WatchDogMode.RESET
# w.feed()


# On CircuitPlayground Express, and boards with built in status NeoPixel -> board.NEOPIXEL
# Otherwise choose an open pin connected to the Data In of the NeoPixel strip, i.e. board.D1
pixel_pin = board.D13


class Scene():
    def __init__(self, wind_input, lighting_output, name):
        self.wind = wind_input
        self.light = lighting_output
        self.name = name
        self.last_wind_presence = False

    def update(self):
        wind_presence = (WIND_VOLTAGE_THRESHOLD < (self.wind.value * 3.3) / 65536)
        if wind_presence and not self.last_wind_presence:
            self.light.value = True
            print(f"{self.name} Scene Turning On: {self.light.value}")
        elif self.last_wind_presence and not wind_presence:
            self.light.value = False
            print(f"{self.name} Scene Turning Off: {self.light.value}")
        # else: nothing changed.
        self.last_wind_presence = wind_presence



class Knob():
    def __init__(self, seesaw, servo):
        self.seesaw = seesaw
        self.servo = servo
        seesaw_product = (self.seesaw.get_version() >> 16) & 0xFFFF
        print(f"Found product {seesaw_product}")
        if seesaw_product != 4991:
            print("Wrong firmware loaded?  Expected 4991")

        self.button = sdio.DigitalIO(seesaw, 24)
        self.button_held = False
        self.encoder = rotaryio.IncrementalEncoder(self.seesaw)
        self.last_position = 0
        self.last_change = 0
        self.azimuth = 0

    def update(self):
        position = -self.encoder.position
        if position > self.last_position:
            # print(f"Position increased to: {position}, diff {position - self.last_position}")
            if ((position - self.last_change) > RESOLUTION):
                # print("ADD "*15)
                self.azimuth += INCREMENT_PER_ENCODER_TICK
                self.last_change = position
            if self.azimuth > UPPER_AZIMUTH_LIMIT:
                # print("Knob attempting to pass upper limit.")
                self.azimuth = UPPER_AZIMUTH_LIMIT
            print(f"Azimuth = {self.azimuth}")
            self.last_position = position
            self.servo.angle = self.azimuth
        elif position < self.last_position:
            # print(f"Position decreased to: {position}, diff {self.last_position - position}")
            if ((self.last_change - position) > RESOLUTION):
                # print("REMOVE "*15)
                self.azimuth -= INCREMENT_PER_ENCODER_TICK
                self.last_change = position
            if self.azimuth < LOWER_AZIMUTH_LIMIT:
                # print("Knob attempting to pass lower limit.")
                self.azimuth = LOWER_AZIMUTH_LIMIT
            print(f"Azimuth = {self.azimuth}")
            self.last_position = position
            self.servo.angle = self.azimuth



        if not self.button.value and not self.button_held:
            self.button_held = True
            print("Button pressed")
        if self.button.value and self.button_held:
            self.button_held = False
            print("Button released")


angle_knob = Knob(seesaw.Seesaw(board.I2C(), addr=0x36), fan_servo)
house_scene = Scene(windmill_b_analog_in, scene_b_pin, "House")
highrise_scene = Scene(windmill_a_analog_in, scene_a_pin, "Highrise")
train_scene = Scene(windmill_c_analog_in, scene_c_pin, "Train")

# if __name__ == "__main__":

print("Boot complete, starting loop...")

while True:
    # w.feed() # Feed the watchdog
    fan_button.update()
    if fan_button.fell:
        fan_led_pin.value = True
        print("Fan button fell, starting fan!")
        fan_speed.angle = 40
    if fan_button.rose:
        fan_led_pin.value = False
        print("Fan button rose, stopping fan!")
        fan_speed.angle = 0
    angle_knob.update()
    house_scene.update()
    highrise_scene.update()
    train_scene.update()

    
Start
