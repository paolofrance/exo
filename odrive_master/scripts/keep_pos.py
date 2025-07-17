import odrive
from odrive.enums import *
import time

# ----------------------------
# ✏️ Choose the axis to control (0 or 1)
CONTROL_AXIS = 0
# ----------------------------

print("Connecting to ODrive...")
odrv = odrive.find_any()
print("Connected.")

# Select axis
axis = odrv.axis0 if CONTROL_AXIS == 0 else odrv.axis1

# Get both axes for monitoring
axis0 = odrv.axis0
axis1 = odrv.axis1

# Read starting position
pos = axis.encoder.pos_estimate
print(f"Axis{CONTROL_AXIS} starting position: {pos}")

# Calibration
print(f"Calibrating Axis{CONTROL_AXIS}...")
axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while axis.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
print("Calibration complete.")

# Set control mode and engage closed loop
axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
time.sleep(0.1)

# Hold position
axis.controller.input_pos = pos
print(f"Holding position on Axis{CONTROL_AXIS}...")

try:
    while True:
        axis.controller.input_pos = pos  # Keep holding
        print("----------")
        print(f"Axis0 | Pos: {axis0.encoder.pos_estimate:.2f}, Current: {axis0.motor.current_control.Iq_measured:.2f} A")
        print(f"Axis1 | Pos: {axis1.encoder.pos_estimate:.2f}, Current: {axis1.motor.current_control.Iq_measured:.2f} A")
        time.sleep(0.2)

except KeyboardInterrupt:
    print("\nStopping motor...")
    axis.requested_state = AXIS_STATE_IDLE
