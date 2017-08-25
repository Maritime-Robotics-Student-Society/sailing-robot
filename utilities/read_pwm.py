"""Experimentation with reading PWM signal on Raspberry Pi.

This technique is used in sensor_driver_multiplexer
"""
import pigpio
import time

GPIO = 22

last_tick = 0
last_interval = 0

def tick(gpio, level, tick):
    global last_tick
    last_tick = tick

def tock(gpio, level, tick):
    global last_interval
    last_interval = tick - last_tick

pi = pigpio.pi()
pi.set_mode(GPIO, pigpio.INPUT)

pi.callback(GPIO, pigpio.RISING_EDGE, tick)
pi.callback(GPIO, pigpio.FALLING_EDGE, tock)

while True:
    time.sleep(1)
    print(last_interval)
