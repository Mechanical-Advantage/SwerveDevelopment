// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.util.SparkMAXBurnManager;

public class ModuleIOSparkMAX implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveRelativeEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final SparkMaxAnalogSensor turnAbsoluteEncoder;

  private final double driveAfterEncoderReduction =
      (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private final double turnAfterEncoderReduction = 150.0 / 7.0;

  private final boolean isDriveMotorInverted = false;
  private final boolean isTurnMotorInverted = false;
  private final boolean isAbsoluteEncoderInverted;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMAX(int index) {
    switch (Constants.getRobot()) {
      case ROBOT_2022S:
        switch (index) {
          case 0:
            driveSparkMax = new CANSparkMax(15, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(11, MotorType.kBrushless);
            isAbsoluteEncoderInverted = false;
            absoluteEncoderOffset = new Rotation2d();
            break;
          case 1:
            driveSparkMax = new CANSparkMax(12, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(9, MotorType.kBrushless);
            isAbsoluteEncoderInverted = false;
            absoluteEncoderOffset = new Rotation2d();
            break;
          case 2:
            driveSparkMax = new CANSparkMax(14, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(10, MotorType.kBrushless);
            isAbsoluteEncoderInverted = false;
            absoluteEncoderOffset = new Rotation2d();
            break;
          case 3:
            driveSparkMax = new CANSparkMax(13, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(8, MotorType.kBrushless);
            isAbsoluteEncoderInverted = false;
            absoluteEncoderOffset = new Rotation2d();
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

    driveSparkMax.setInverted(isDriveMotorInverted);
    turnSparkMax.setInverted(isTurnMotorInverted);

    driveSparkMax.setSmartCurrentLimit(30);
    turnSparkMax.setSmartCurrentLimit(30);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    turnAbsoluteEncoder = turnSparkMax.getAnalog(Mode.kAbsolute);
    turnAbsoluteEncoder.setInverted(isAbsoluteEncoderInverted);

    driveRelativeEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();
    driveRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setPosition(0.0);

    turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 20);

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
        Units.rotationsToRadians(driveRelativeEncoder.getPosition())
            / driveAfterEncoderReduction;
    inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        driveRelativeEncoder.getVelocity()) / driveAfterEncoderReduction;
    inputs.driveAppliedVolts =
        driveSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};
    inputs.driveTempCelcius =
        new double[] {driveSparkMax.getMotorTemperature()};

    inputs.turnAbsolutePositionRad = new Rotation2d(
        (turnAbsoluteEncoder.getPosition() / 3.3) * 2.0 * Math.PI)
            .minus(absoluteEncoderOffset).getRadians();
    inputs.turnPositionRad =
        Units.rotationsToRadians(turnAbsoluteEncoder.getPosition())
            / turnAfterEncoderReduction;
    inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        turnAbsoluteEncoder.getVelocity()) / turnAfterEncoderReduction;
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
