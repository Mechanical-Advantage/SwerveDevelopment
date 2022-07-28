// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Add your docs here. */
public interface GyroIO {
  public static class GyroIOInputs implements LoggableInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;

    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("PositionRad", positionRad);
      table.put("VelocityRadPerSec", velocityRadPerSec);
    }

    public void fromLog(LogTable table) {
      connected = table.getBoolean("Connected", connected);
      positionRad = table.getDouble("PositionRad", positionRad);
      velocityRadPerSec =
          table.getDouble("VelocityRadPerSec", velocityRadPerSec);
    }
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
