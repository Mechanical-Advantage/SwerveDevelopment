// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.util.SparkMAXBurnManager;
import frc.robot.util.SparkMaxDerivedVelocityController;

public class ModuleIOSparkMAX implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final SparkMaxDerivedVelocityController driveDerivedVelocityController;
  private final RelativeEncoder driveDefaultEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final AnalogInput turnAbsoluteEncoder;

  private final double driveAfterEncoderReduction =
      (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private final double turnAfterEncoderReduction = 150.0 / 7.0;

  private final boolean isTurnMotorInverted = true;
  private final boolean isAbsoluteEncoderInverted = false;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMAX(int index) {
    switch (Constants.getRobot()) {
      case ROBOT_2022S:
        switch (index) {
          case 0:
            driveSparkMax = new CANSparkMax(15, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(11, MotorType.kBrushless);
            turnAbsoluteEncoder = new AnalogInput(0);
            absoluteEncoderOffset = new Rotation2d(-0.036);
            break;
          case 1:
            driveSparkMax = new CANSparkMax(12, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(9, MotorType.kBrushless);
            turnAbsoluteEncoder = new AnalogInput(1);
            absoluteEncoderOffset = new Rotation2d(1.0185);
            break;
          case 2:
            driveSparkMax = new CANSparkMax(14, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(10, MotorType.kBrushless);
            turnAbsoluteEncoder = new AnalogInput(2);
            absoluteEncoderOffset = new Rotation2d(1.0705);
            break;
          case 3:
            driveSparkMax = new CANSparkMax(13, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(8, MotorType.kBrushless);
            turnAbsoluteEncoder = new AnalogInput(3);
            absoluteEncoderOffset = new Rotation2d(0.7465);
            break;
          default:
            throw new RuntimeException(
                "Invalid module index for ModuleIOSparkMAX");
        }
        break;
      default:
        throw new RuntimeException("Invalid robot for ModuleIOSparkMAX");
    }

    if (SparkMAXBurnManager.shouldBurn()) {
      driveSparkMax.restoreFactoryDefaults();
      turnSparkMax.restoreFactoryDefaults();
    }

    turnSparkMax.setInverted(isTurnMotorInverted);

    driveSparkMax.setSmartCurrentLimit(30);
    turnSparkMax.setSmartCurrentLimit(30);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    driveDerivedVelocityController =
        new SparkMaxDerivedVelocityController(driveSparkMax);
    driveDefaultEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnRelativeEncoder.setPosition(0.0);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    if (SparkMAXBurnManager.shouldBurn()) {
      driveSparkMax.burnFlash();
      turnSparkMax.burnFlash();
    }
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveDerivedVelocityController.getPosition())
            / driveAfterEncoderReduction;
    inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        driveDerivedVelocityController.getVelocity())
        / driveAfterEncoderReduction;
    inputs.driveVelocityFilteredRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            driveDefaultEncoder.getVelocity()) / driveAfterEncoderReduction;
    inputs.driveAppliedVolts =
        driveSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};
    inputs.driveTempCelcius =
        new double[] {driveSparkMax.getMotorTemperature()};

    double absolutePositionPercent =
        turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    if (isAbsoluteEncoderInverted) {
      absolutePositionPercent = 1 - absolutePositionPercent;
    }
    inputs.turnAbsolutePositionRad =
        new Rotation2d(absolutePositionPercent * 2.0 * Math.PI)
            .minus(absoluteEncoderOffset).getRadians();
    inputs.turnPositionRad =
        Units.rotationsToRadians(turnRelativeEncoder.getPosition())
            / turnAfterEncoderReduction;
    inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        turnRelativeEncoder.getVelocity()) / turnAfterEncoderReduction;
    inputs.turnAppliedVolts =
        turnSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
    inputs.turnTempCelcius = new double[] {turnSparkMax.getMotorTemperature()};
  }

  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
