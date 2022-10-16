// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import frc.robot.Constants;

public class ClimberIOTalonSRX implements ClimberIO {
  private final TalonSRX motor;
  private boolean invert = false;

  public ClimberIOTalonSRX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022S:
        motor = new TalonSRX(3);
        invert = true;
        break;
      default:
        throw new RuntimeException("Invalid robot for ClimberIOSparkMAX!");
    }

    TalonSRXConfiguration config = new TalonSRXConfiguration();
    config.voltageCompSaturation = 12.0;
    config.peakCurrentLimit = 10;
    motor.configAllSettings(config);
    motor.setInverted(invert);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVolts = motor.getMotorOutputVoltage();
    inputs.currentAmps = new double[] {motor.getSupplyCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    motor.set(ControlMode.PercentOutput, volts / 12.0);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    motor.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
  }
}
