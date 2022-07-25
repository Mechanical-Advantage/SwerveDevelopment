// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/** Drive subsystem hardware interface for a swerve module (using WPILib's flywheel sim). */
public class ModuleIOSim implements ModuleIO {
  private FlywheelSim driveSim =
      new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
  private FlywheelSim turnSim =
      new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004096955);

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(Constants.loopPeriodSecs);
    turnSim.update(Constants.loopPeriodSecs);

    inputs.drivePositionRad = inputs.drivePositionRad
        + (driveSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps =
        new double[] {Math.abs(driveSim.getCurrentDrawAmps())};
    inputs.driveTempCelcius = new double[] {};

    inputs.turnPositionRad = inputs.turnPositionRad
        + (turnSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps =
        new double[] {Math.abs(turnSim.getCurrentDrawAmps())};
    inputs.turnTempCelcius = new double[] {};
  }

  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }
}
