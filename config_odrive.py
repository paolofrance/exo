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

    ''' Set motor parameters '''
    # Current limit [A]
    odrv0.axis0.motor.config.current_lim = 45.0 
    odrv0.axis1.motor.config.current_lim = 45.0 
    # Calibration current [A]
    odrv0.axis0.motor.config.calibration_current = 15.0
    odrv0.axis1.motor.config.calibration_current = 15.0
    # Torque constant [Nm/A]
    odrv0.axis0.motor.config.torque_constant = 0.0250606
    odrv0.axis1.motor.config.torque_constant = 0.0250606
    # Pole pairs
    odrv0.axis0.motor.config.pole_pairs = 7
    odrv0.axis1.motor.config.pole_pairs = 7
    # Torque limit [Nm]
    odrv0.axis0.motor.config.torque_lim = 1.5
    odrv0.axis1.motor.config.torque_lim = 1.5

    sleep(0.1)

    ''' Set controller parameters '''
    # Control mode (torque control)
    odrv0.axis0.controller.config.control_mode = 1
    odrv0.axis1.controller.config.control_mode = 1
    # Input mode (CUSTOM)
    odrv0.axis0.controller.config.input_mode = 9
    odrv0.axis1.controller.config.input_mode = 9
    # Velocity limit [turn/s]
    odrv0.axis0.controller.config.vel_limit = 10
    odrv0.axis1.controller.config.vel_limit = 10

    # Controller gains (tune as needed)
    odrv0.axis0.controller.config.pos_gain = 20.0
    odrv0.axis0.controller.config.vel_gain = 0.16666667
    odrv0.axis0.controller.config.vel_integrator_gain = 0.33333334

    odrv0.axis1.controller.config.pos_gain = 20.0
    odrv0.axis1.controller.config.vel_gain = 0.16666667
    odrv0.axis1.controller.config.vel_integrator_gain = 0.33333334

    sleep(0.1)

    ''' Encoder parameters '''
    odrv0.config.gpio5_mode = 0
    odrv0.config.gpio6_mode = 0
    odrv0.axis0.encoder.config.abs_spi_cs_gpio_pin = 5
    odrv0.axis1.encoder.config.abs_spi_cs_gpio_pin = 6
    # CPR
    odrv0.axis0.encoder.config.cpr = 16384
    odrv0.axis1.encoder.config.cpr = 16384
    # Mode (SPI AMS)
    odrv0.axis0.encoder.config.mode = 257
    odrv0.axis1.encoder.config.mode = 257

    sleep(0.1)

    ''' SEA torque sensor '''
    # odrv0.axis0.torque_sensor.config.enable = False
    # odrv0.axis1.torque_sensor.config.enable = False

    ans = input("Config ok!\nSave configuration? [y/N]")
    if ans.upper() == "Y":
        odrv0.save_configuration()




