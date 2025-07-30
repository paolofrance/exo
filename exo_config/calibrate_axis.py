#!/usr/bin/env python3
import odrive
from odrive.enums import *
import time

def wait_for_idle(axis):
    print("⏳ Waiting for calibration to complete...")
    while axis.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

def calibrate_axis(odrv, axis_num):
    axis = odrv.axis0 if axis_num == 0 else odrv.axis1
    print(f"🧪 Starting calibration for Axis {axis_num}...")

    # Clear errors
    axis.error = 0
    axis.motor.error = 0
    axis.encoder.error = 0

    # Start calibration
    axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    wait_for_idle(axis)

    # Check results
    if axis.motor.error != 0 or axis.encoder.error != 0 or axis.error != 0:
        print(f"❌ Calibration failed for Axis {axis_num}")
        print(f"  Axis error: {axis.error}")
        print(f"  Motor error: {axis.motor.error}")
        print(f"  Encoder error: {axis.encoder.error}")
        return False

    # Mark as pre-calibrated
    axis.motor.config.pre_calibrated = True
    axis.encoder.config.pre_calibrated = True
    return True

def main():
    print("🔌 Searching for ODrive...")
    odrv = odrive.find_any()
    print(f"✅ Connected to ODrive serial: {odrv.serial_number}")
    print(f"🔋 Bus voltage: {odrv.vbus_voltage:.2f} V")

    axis_num = int(input("🧭 Enter axis to calibrate (0 or 1): "))
    success = calibrate_axis(odrv, axis_num)

    if success:
        print("💾 Saving configuration...")
        odrv.save_configuration()
        print("🔄 Rebooting...")
        try:
            odrv.reboot()
        except:
            print("✅ Rebooted (device will disconnect, this is expected).")
    else:
        print("⚠️ Calibration failed. Configuration not saved.")

if __name__ == '__main__':
    main()
