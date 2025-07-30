import odrive
import time

# Connect to ODrive (waits until found)
print("Looking for ODrive...")
odrv0 = odrive.find_any()
print("ODrive found.")

try:
    while True:
        # Read encoder positions
        pos0 = odrv0.axis0.encoder.pos_estimate
        pos1 = odrv0.axis1.encoder.pos_estimate

        print(f"Axis0 Position: {pos0:.3f} turns | Axis1 Position: {pos1:.3f} turns")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped by user.")
