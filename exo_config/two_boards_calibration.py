#!/usr/bin/env python3

import odrive
from odrive.enums import *
import time

# ODRIVE_SERIAL = "317532613431"
ODRIVE_SERIAL = "365A388C3131"

def wait_for_idle(axis, timeout=30.0):
    start_time = time.time()
    while axis.current_state != AXIS_STATE_IDLE:
        if time.time() - start_time > timeout:
            print("❌ Timeout waiting for axis to become IDLE.")
            return False
        time.sleep(0.1)
    return True

def calibrate_axis0(odrv, label):
    print(f"\n🔧 Calibrating {label} axis0")
    axis = odrv.axis0

    odrv.clear_errors()
    time.sleep(0.1)

    axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    if not wait_for_idle(axis):
        print(f"❌ {label} axis0 did not return to IDLE.")
        return False

    if axis.error or axis.motor.error or axis.encoder.error:
        print(f"❌ {label} axis0 encountered errors:")
        print(f"  Axis error:    {axis.error}")
        print(f"  Motor error:   {axis.motor.error}")
        print(f"  Encoder error: {axis.encoder.error}")
        return False

    axis.motor.config.pre_calibrated = True
    axis.encoder.config.pre_calibrated = True
    print(f"✅ {label} axis0 calibration successful")

    print(f"💾 {label}: Saving configuration...")
    odrv.save_configuration()
    print(f"✅ {label}: Configuration saved")
    return True

def main():
    print(f"🔍 Connecting to ODrive 1 ({ODRIVE_SERIAL})...")
    odrv1 = odrive.find_any(serial_number=ODRIVE_SERIAL)
    print("✅ ODrive 1 connected")

    ok1 = calibrate_axis0(odrv1, "ODrive 1")

    if ok1:
        print("\n🎉 axis0 calibrated on both ODrives.")
    else:
        print("\n⚠️ Calibration failed on one or both ODrives.")

if __name__ == "__main__":
    main()
