// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class ModuleIOSim implements ModuleIO {
  private FlywheelSim driveSim =
      new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
  private FlywheelSim turnSim =
      new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004096955);

  private double turnRelativePositionRad = 0.0;
  private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(Constants.loopPeriodSecs);
    turnSim.update(Constants.loopPeriodSecs);

    double angleDiffRad =
        turnSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs;
    turnRelativePositionRad += angleDiffRad;
    turnAbsolutePositionRad += angleDiffRad;
    while (turnAbsolutePositionRad < 0) {
      turnAbsolutePositionRad += 2.0 * Math.PI;
    }
    while (turnAbsolutePositionRad > 2.0 * Math.PI) {
      turnAbsolutePositionRad -= 2.0 * Math.PI;
    }

    inputs.drivePositionRad = inputs.drivePositionRad
        + (driveSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps =
        new double[] {Math.abs(driveSim.getCurrentDrawAmps())};
    inputs.driveTempCelcius = new double[] {};

    inputs.turnAbsolutePositionRad = turnAbsolutePositionRad;
    inputs.turnPositionRad = turnRelativePositionRad;
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
