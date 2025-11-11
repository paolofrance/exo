#!/usr/bin/python3 -B

import asyncio
import moteus
import sys
import pprint

# --- Diagnostic prints ---
print("--- Python and Moteus Diagnostics ---")
print(f"Python executable: {sys.executable}")
print("Python search path (sys.path):")
pprint.pprint(sys.path)
try:
    # We need to import moteus to check its properties
    print(f"Moteus library version: {moteus.__version__}")
    print(f"Moteus library path: {moteus.__file__}")
except AttributeError:
    # This block runs if __version__ doesn't exist, confirming an old version
    print("Could not determine moteus version. It is likely very old.")
    print(f"The conflicting Moteus library is located at: {moteus.__file__}")
print("-------------------------------------\n")

async def main():
    # Connect to motor ID 1 on default FDCAN-USB transport
    transport = moteus.Fdcanusb()
    c = moteus.Controller(id=1, transport=transport)

    # Stop motor before applying config
    await c.set_stop()

    # --- Configuration parameters ---
    config = {
        # Position limits in turns
        "servo.max_position": 10.0,
        "servo.min_position": -10.0,

        # Trajectory limits
        "servo.velocity_limit": 5.0,      # turns/sec
        "servo.acceleration_limit": 10.0, # turns/sec^2

        # Slip limits
        "servo.max_position_slip": 0.2,   # turns
        "servo.max_velocity_slip": 2.0,   # turns/sec

        # Optional gain scales (1.0 = default)
        "servo.kp_scale": 1.0,
        "servo.kd_scale": 0.5,
        "servo.i_scale": 0.0,
    }

    # Apply all parameters from the config dictionary at once
    print("Applying configuration...")
    await c.set_conf(config)

    # Save to flash so settings persist across power cycles
    print("Saving configuration to flash...")
    await c.write_config()
    print("✅ Configuration saved!")

if __name__ == "__main__":
    asyncio.run(main())
