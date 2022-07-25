// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Drive subsystem hardware interface for a gyro. */
public interface GyroIO {
  /** The set of loggable inputs for the drive subsystem. */
  public static class GyroIOInputs implements LoggableInputs {
    public boolean connected = false;
    public double yawPositionRad = 0.0;
    public double yawVelocityRadPerSec = 0.0;

    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("YawPositionRad", yawPositionRad);
      table.put("YawVelocityRadPerSec", yawVelocityRadPerSec);
    }

    public void fromLog(LogTable table) {
      connected = table.getBoolean("Connected", connected);
      yawPositionRad = table.getDouble("YawPositionRad", yawPositionRad);
      yawVelocityRadPerSec =
          table.getDouble("YawVelocityRadPerSec", yawVelocityRadPerSec);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GyroIOInputs inputs) {}
}
