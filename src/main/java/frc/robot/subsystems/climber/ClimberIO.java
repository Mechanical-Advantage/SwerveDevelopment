// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** Climber subsystem hardware interface. */
public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Run the roller open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Enable or disable brake mode on the roller. */
  public default void setBrakeMode(boolean enable) {}
}
