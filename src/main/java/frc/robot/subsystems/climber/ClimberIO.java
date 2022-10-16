// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Climber subsystem hardware interface. */
public interface ClimberIO {
  /** Contains all of the input data received from hardware. */
  public static class ClimberIOInputs implements LoggableInputs {
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};

    public void toLog(LogTable table) {
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
    }

    public void fromLog(LogTable table) {
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Run the roller open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Enable or disable brake mode on the roller. */
  public default void setBrakeMode(boolean enable) {}
}
