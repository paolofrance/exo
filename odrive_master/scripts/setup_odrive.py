#!/usr/bin/env python3

import odrive
from odrive.enums import *
from time import sleep

def connect_odrive():
    odrv0 = odrive.find_any()
    print("Connected to {} as odrv0".format(odrv0.serial_number))
    print("Battery voltage: {:2.2f} V".format(odrv0.vbus_voltage))
    return odrv0

if __name__ == '__main__':
    
    # Connect ODrive
    odrv0 = connect_odrive()

    print("\n--- Configuring Motor Parameters ---")
    for axis in [odrv0.axis0, odrv0.axis1]:
        axis.motor.config.current_lim = 45.0               # Amps
        axis.motor.config.calibration_current = 15.0       # Amps for calibration
        axis.motor.config.torque_constant = 0.0250606      # Nm/A for MJ5208
        axis.motor.config.pole_pairs = 7                   # MJ5208 has 14 poles = 7 pairs
        axis.motor.config.resistance_calib_max_voltage = 4
        axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT

    print("\n--- Configuring Controller Parameters ---")
    for axis in [odrv0.axis0, odrv0.axis1]:
        axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        axis.controller.config.vel_limit = 10              # turns/sec

    print("\n--- Configuring SPI Encoder Parameters ---")
    # GPIO CS pins (check your wiring!)
    odrv0.config.gpio5_mode = GPIO_MODE_DIGITAL
    odrv0.config.gpio6_mode = GPIO_MODE_DIGITAL
    odrv0.axis0.encoder.config.abs_spi_cs_gpio_pin = 5
    odrv0.axis1.encoder.config.abs_spi_cs_gpio_pin = 6

    for axis in [odrv0.axis0, odrv0.axis1]:
        axis.encoder.config.mode = 257   # SPI Absolute mode
        axis.encoder.config.cpr = 16384
        axis.encoder.config.bandwidth = 1000
        axis.encoder.config.use_index = False
        axis.encoder.config.ignore_illegal_hall_state = True
        axis.encoder.config.pre_calibrated = False

    print("\n--- Done. ---")
    ans = input("Save and reboot? [y/N]: ").strip().lower()
    if ans == "y":
        print("Saving configuration...")
        odrv0.save_configuration()
        print("Rebooting...")
        odrv0.reboot()
        print("Waiting for reconnect...")
        sleep(5)
        odrv0 = odrive.find_any()
        print("✅ Reconnected to {}".format(odrv0.serial_number))
    else:
        print("⚠️ Changes not saved.")
