# SPDX-FileCopyrightText: 2021 John Furcean
# SPDX-License-Identifier: MIT

"""I2C rotary encoders in array of Knobs."""

import time
from random import choice
from time import sleep
import board
from adafruit_seesaw import seesaw, rotaryio
from adafruit_seesaw import digitalio as sdio
import neopixel
import digitalio as dio
from adafruit_debouncer import Debouncer
from microcontroller import watchdog as w
from watchdog import WatchDogMode

import pwmio
from adafruit_motor import servo

RESOLUTION = 0  # Only change on every nth position.
INCREMENT_PER_ENCODER_TICK = 2
UPPER_AZIMUTH_LIMIT = 180
LOWER_AZIMUTH_LIMIT = 0

# create a PWMOut object on Pin A15.
pwm = pwmio.PWMOut(board.A15, duty_cycle=2 ** 15, frequency=50)

# Create a servo object, my_servo.
fan_servo = servo.Servo(pwm)


w.timeout=4 # Set a timeout of 2.5 seconds
w.mode = WatchDogMode.RESET
w.feed()


lever_pin = dio.DigitalInOut(board.D10)
lever_pin.direction = dio.Direction.INPUT
lever_pin.pull = dio.Pull.UP
lever = Debouncer(lever_pin)

sound_trigger_pin = dio.DigitalInOut(board.D9)
sound_trigger_pin.direction = dio.Direction.OUTPUT
sound_trigger_pin.drive_mode = dio.DriveMode.OPEN_DRAIN
sound_trigger_pin.value = True


# On CircuitPlayground Express, and boards with built in status NeoPixel -> board.NEOPIXEL
# Otherwise choose an open pin connected to the Data In of the NeoPixel strip, i.e. board.D1
pixel_pin = board.D13

# On a Raspberry pi, use this instead, not all pins are supported
# pixel_pin = board.D18




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

# if __name__ == "__main__":

print("Boot complete, starting loop...")

while True:
    w.feed() # Feed the watchdog
    lever.update()
    if lever.rose or lever.fell:
        print("Lever changed!")

    angle_knob.update()
