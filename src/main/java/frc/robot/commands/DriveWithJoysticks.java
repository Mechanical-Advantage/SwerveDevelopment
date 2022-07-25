// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;
import frc.robot.util.TunableNumber;

public class DriveWithJoysticks extends CommandBase {
  private static final TunableNumber deadband =
      new TunableNumber("DriveWithJoysticks/Deadband", 0.1);

  private final Drive drive;
  private final Supplier<Double> xVelocityAxis;
  private final Supplier<Double> yVelocityAxis;
  private final Supplier<Double> omegaVelocityAxis;

  private boolean isFieldRelative = true;
  private boolean autoAim = false;
  private PIDController autoAimController =
      new PIDController(15.0, 0.0, 0.0, Constants.loopPeriodSecs);

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(Drive drive, Supplier<Double> xVelocityAxis,
      Supplier<Double> yVelocityAxis, Supplier<Double> omegaVelocityAxis) {
    addRequirements(drive);
    this.drive = drive;
    this.xVelocityAxis = xVelocityAxis;
    this.yVelocityAxis = yVelocityAxis;
    this.omegaVelocityAxis = omegaVelocityAxis;

    autoAimController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double driveMagnitude =
        Math.hypot(xVelocityAxis.get(), yVelocityAxis.get());
    if (Math.abs(driveMagnitude) > deadband.get()) {
      driveMagnitude =
          (Math.abs(driveMagnitude) - deadband.get()) / (1 - deadband.get());
      driveMagnitude = driveMagnitude * driveMagnitude;
    } else {
      driveMagnitude = 0.0;
    }
    Translation2d driveVelocity = new Pose2d(new Translation2d(),
        new Rotation2d(xVelocityAxis.get(), yVelocityAxis.get()))
            .transformBy(GeomUtil.transformFromTranslation(driveMagnitude, 0.0))
            .getTranslation();

    double omegaVelocityPercent = processSingleAxis(omegaVelocityAxis.get());

    Logger.getInstance().recordOutput("DriveWithJoysticks/IsFieldRelative",
        isFieldRelative);
    Logger.getInstance().recordOutput("DriveWithJoysticks/XVelocityPercent",
        driveVelocity.getX());
    Logger.getInstance().recordOutput("DriveWithJoysticks/YVelocityPercent",
        driveVelocity.getY());
    Logger.getInstance().recordOutput("DriveWithJoysticks/OmegaVelocityPercent",
        omegaVelocityPercent);

    ChassisSpeeds speeds = new ChassisSpeeds(
        driveVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
        driveVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
        omegaVelocityPercent * drive.getMaxAngularSpeedRadPerSec());

    if (autoAim) {
      speeds.omegaRadiansPerSecond = autoAimController.calculate(
          drive.getRotation().getRadians(),
          GeomUtil.direction(
              FieldConstants.hubCenter.minus(drive.getPose().getTranslation()))
              .getRadians());
    } else {
      autoAimController.reset();
    }

    if (isFieldRelative) {
      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
          speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
          speeds.omegaRadiansPerSecond, drive.getRotation()));
    } else {
      drive.runVelocity(speeds);
    }
  }

  public double processSingleAxis(double value) {
    double output = 0.0;
    if (Math.abs(value) > deadband.get()) {
      output = (Math.abs(value) - deadband.get()) / (1 - deadband.get());
      output = Math.copySign(output * output, value);
    }
    return output;
  }

  public void toggleFieldRelative() {
    isFieldRelative = !isFieldRelative;
  }

  public void setAutoAim(boolean enabled) {
    autoAim = enabled;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
