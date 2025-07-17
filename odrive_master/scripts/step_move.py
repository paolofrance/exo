import odrive
from odrive.enums import *
import time

print("Connecting to ODrive...")
odrv = odrive.find_any()
print(f"Connected to ODrive ({odrv.serial_number})")

# Function to clear, calibrate and check an axis
def initialize_axis(axis, name="axis"):
    print(f"\n--- Initializing {name} ---")
    odrv.clear_errors()
    axis.error = 0
    axis.motor.error = 0
    axis.encoder.error = 0

    print(f"Starting calibration on {name}...")
    axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    while axis.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    if axis.error != 0 or axis.motor.error != 0 or axis.encoder.error != 0:
        print(f"❌ {name} error after calibration!")
        print("Axis error:    ", axis.error)
        print("Motor error:   ", axis.motor.error)
        print("Encoder error: ", axis.encoder.error)
        exit(1)

    if not axis.encoder.is_ready:
        print(f"❌ {name} encoder not ready!")
        exit(1)

    print(f"{name} calibration OK.")

    print(f"Entering closed-loop control for {name}...")
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.1)

    if axis.error != 0 or axis.motor.error != 0 or axis.encoder.error != 0:
        print(f"❌ {name} error entering closed-loop!")
        print("Axis error:    ", axis.error)
        print("Motor error:   ", axis.motor.error)
        print("Encoder error: ", axis.encoder.error)
        exit(1)

    print(f"{name} is in closed-loop control.")

# Initialize both axes
initialize_axis(odrv.axis0, "axis0")
initialize_axis(odrv.axis1, "axis1")

# Ask which motor to move
while True:
    choice = input("\nWhich motor do you want to move? [0 or 1]: ").strip()
    if choice in ["0", "1"]:
        CONTROL_AXIS = int(choice)
        break
    print("Invalid input. Please enter 0 or 1.")

axis_to_move = odrv.axis0 if CONTROL_AXIS == 0 else odrv.axis1
print(f"\nSelected axis{CONTROL_AXIS} for movement.")

# Move selected motor to 0 position
print("Moving selected motor to 0.0 turns...")
axis_to_move.controller.input_pos = 0.0
time.sleep(2.0)

# Smooth ramp to 2.0 turns
print("Ramping to 2.0 turns over 5 seconds...")
steps = 100
duration = 5.0  # seconds
for i in range(steps + 1):
    pos = 2.0 * (i / steps)
    axis_to_move.controller.input_pos = pos
    print(f"Setpoint: {pos:.2f} turns")
    time.sleep(duration / steps)

print(f"✅ axis{CONTROL_AXIS} reached 2.0 turns.")

# Optional: IDLE everything at the end
input("\nPress Enter to IDLE both motors...")
odrv.axis0.requested_state = AXIS_STATE_IDLE
odrv.axis1.requested_state = AXIS_STATE_IDLE
print("🛑 Both motors set to IDLE.")
