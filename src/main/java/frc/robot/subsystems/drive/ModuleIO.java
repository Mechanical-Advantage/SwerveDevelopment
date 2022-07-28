// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Add your docs here. */
public interface ModuleIO {
  public static class ModuleIOInputs implements LoggableInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};
    public double[] driveTempCelcius = new double[] {};
    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double[] {};
    public double[] turnTempCelcius = new double[] {};

    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("DrivePositionRad", positionRad);
      table.put("VelocityRadPerSec", velocityRadPerSec);
    }

    public void fromLog(LogTable table) {
      connected = table.getBoolean("Connected", connected);
      positionRad = table.getDouble("PositionRad", positionRad);
      velocityRadPerSec =
          table.getDouble("VelocityRadPerSec", velocityRadPerSec);
    }
  }

  public default void updateInputs(ModuleIOInputs inputs) {}
}
