import odrive
from odrive.enums import *
import time

CONTROL_AXIS = 0



print("Connecting to ODrive...")
odrv = odrive.find_any()
print("Connected.")


axis = odrv.axis0 if CONTROL_AXIS == 0 else odrv.axis1



motor = axis.motor
encoder = axis.encoder

# Show firmware version
print(f"ODrive firmware version: {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")

# Clear previous errors
axis.error = 0
motor.error = 0
encoder.error = 0
print("Cleared errors.")

# Print motor configuration
print("\nMotor Configuration:")
print(f"Pole Pairs: {motor.config.pole_pairs}")
print(f"Calibration Current: {motor.config.calibration_current} A")
print(f"Motor Type: {motor.config.motor_type}")

# Print encoder configuration
print("\nEncoder Configuration:")
print(f"CPR: {encoder.config.cpr}")
print(f"Use Index: {encoder.config.use_index}")
print(f"Mode: {encoder.config.mode}")

# Lower calibration current if motor is small
motor.config.calibration_current = 5  # Amps; lower this if needed

# Start full calibration
print("\nStarting calibration...")
axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

# Wait for it to finish
while axis.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

print("Calibration done.")

# Check for errors
if motor.error != 0:
    print(f"\n❌ Motor Error: {motor.error}")
else:
    print("✅ Motor calibrated without error.")

if encoder.error != 0:
    print(f"⚠️ Encoder Error: {encoder.error}")
else:
    print("✅ Encoder OK.")

# Try to enter closed-loop control
print("\nTrying to enter closed-loop control...")
axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
time.sleep(0.5)

# Set a test position
start_pos = encoder.pos_estimate
axis.controller.input_pos = start_pos + 1.0
print("Sent test move +1 rev")

time.sleep(2)
print(f"Current position: {encoder.pos_estimate:.2f}")

# Check again for errors
if motor.error != 0:
    print(f"\n❌ Motor Error: {motor.error}")
if encoder.error != 0:
    print(f"\n⚠️ Encoder Error: {encoder.error}")
if axis.error != 0:
    print(f"\n🔴 Axis Error: {axis.error}")

print("Done.")
