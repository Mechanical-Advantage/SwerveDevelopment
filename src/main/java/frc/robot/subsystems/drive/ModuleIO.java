// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Drive subsystem hardware interface for a swerve module. */
public interface ModuleIO {
  /** The set of loggable inputs for the drive subsystem. */
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
      table.put("DrivePositionRad", drivePositionRad);
      table.put("DriveVelocityRadPerSec", driveVelocityRadPerSec);
      table.put("DriveAppliedVolts", driveAppliedVolts);
      table.put("DriveCurrentAmps", driveCurrentAmps);
      table.put("DriveTempCelcius", driveTempCelcius);

      table.put("TurnPositionRad", turnPositionRad);
      table.put("TurnVelocityRadPerSec", turnVelocityRadPerSec);
      table.put("TurnAppliedVolts", turnAppliedVolts);
      table.put("TurnCurrentAmps", turnCurrentAmps);
      table.put("TurnTempCelcius", turnTempCelcius);
    }

    public void fromLog(LogTable table) {
      drivePositionRad = table.getDouble("DrivePositionRad", drivePositionRad);
      driveVelocityRadPerSec =
          table.getDouble("DriveVelocityRadPerSec", driveVelocityRadPerSec);
      driveAppliedVolts =
          table.getDouble("DriveAppliedVolts", driveAppliedVolts);
      driveCurrentAmps =
          table.getDoubleArray("DriveCurrentAmps", driveCurrentAmps);
      driveTempCelcius =
          table.getDoubleArray("DriveTempCelcius", driveTempCelcius);

      turnPositionRad = table.getDouble("TurnPositionRad", turnPositionRad);
      turnVelocityRadPerSec =
          table.getDouble("TurnVelocityRadPerSec", turnVelocityRadPerSec);
      turnAppliedVolts = table.getDouble("TurnAppliedVolts", turnAppliedVolts);
      turnCurrentAmps =
          table.getDoubleArray("TurnCurrentAmps", turnCurrentAmps);
      turnTempCelcius =
          table.getDoubleArray("TurnTempCelcius", turnTempCelcius);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Run the turn motor at the specified voltage. */
  public default void setTurnVoltage(double volts) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}
}
