import odrive
import time

# Connect to ODrive
print("Connecting to ODrive...")
odrv0 = odrive.find_any()
print("Connected.")

try:
    while True:
        # Read encoder positions (in turns)
        pos0 = odrv0.axis0.encoder.pos_estimate
        pos1 = odrv0.axis1.encoder.pos_estimate

        # Read motor current (Iq_measured in Amperes)
        current0 = odrv0.axis0.motor.current_control.Iq_measured
        current1 = odrv0.axis1.motor.current_control.Iq_measured

        print(f"Pos0: {pos0:.3f} Pos1: {pos1:.3f} turns | Cur0: {current0:.3f} Cur1 {current1:.3f} A")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped by user.")
