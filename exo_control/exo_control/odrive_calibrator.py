#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import odrive
from odrive.enums import *
import time

class ODriveCalibratorNode(Node):
    def __init__(self):
        super().__init__('odrive_calibrator')
        self.get_logger().info("🔍 Finding an ODrive...")
        self.odrv = odrive.find_any(timeout=10)
        if self.odrv is None:
            self.get_logger().error("❌ No ODrive found. Exiting.")
            rclpy.shutdown()
            return
        self.get_logger().info("✅ ODrive connected!")

        success = self.calibrate_and_save(self.odrv)
        if success:
            self.get_logger().info("🎉 Calibration and saving complete. No need to recalibrate next time.")
        else:
            self.get_logger().warn("⚠️ Calibration failed. Please check motor wiring and try again.")
        rclpy.shutdown()

    def wait_for_idle(self, axis, timeout=30.0):
        start_time = time.time()
        while rclpy.ok():
            if axis.current_state == AXIS_STATE_IDLE:
                return True
            if time.time() - start_time > timeout:
                self.get_logger().error("❌ Timeout waiting for axis to become IDLE.")
                return False
            time.sleep(0.1)
        return False

    def calibrate_and_save(self, odrv):
        for i in [0, 1]:
            axis = odrv.axis0 if i == 0 else odrv.axis1
            self.get_logger().info(f"➡️ Starting calibration for axis {i}...")

            # Clear previous errors
            odrv.clear_errors()
            time.sleep(0.1)

            # Start calibration
            axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            if not self.wait_for_idle(axis):
                self.get_logger().error(f"❌ Axis {i} did not return to IDLE.")
                return False

            # Check errors
            if axis.error != 0 or axis.motor.error != 0 or axis.encoder.error != 0:
                self.get_logger().error(f"❌ Calibration failed on axis {i}!")
                self.get_logger().error(f"Axis error:    {axis.error}")
                self.get_logger().error(f"Motor error:   {axis.motor.error}")
                self.get_logger().error(f"Encoder error: {axis.encoder.error}")
                return False

            self.get_logger().info(f"✅ Axis {i} calibration successful!")

            # Mark as pre-calibrated
            axis.motor.config.pre_calibrated = True
            axis.encoder.config.pre_calibrated = True

        self.get_logger().info("💾 Saving configuration to flash...")
        odrv.save_configuration()
        self.get_logger().info("✅ Configuration saved successfully!")
        return True

def main(args=None):
    rclpy.init(args=args)
    node = ODriveCalibratorNode()
    rclpy.spin(node)  # Will exit immediately after calibration & shutdown called
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
