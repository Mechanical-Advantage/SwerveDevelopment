// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.Supplier;

public class DriveWithJoysticks extends CommandBase {

  private final Drive drive;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightYSupplier;
  private final Supplier<Boolean> robotRelativeOverride;
  private final Supplier<Double> speedLimitSupplier;

  private static final double deadband = 0.1;

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(Drive drive, Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier, Supplier<Double> rightYSupplier,
      Supplier<Boolean> robotRelativeOverride,
      Supplier<Double> speedLimitSuplier) {
    addRequirements(drive);
    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightYSupplier = rightYSupplier;
    this.robotRelativeOverride = robotRelativeOverride;
    this.speedLimitSupplier = speedLimitSuplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get values from double suppliers
    double leftX = leftXSupplier.get();
    double leftY = leftYSupplier.get();
    double rightY = rightYSupplier.get();

    // Get direction and magnitude of linear axes
    double linearMagnitude = Math.hypot(leftX, leftY);
    Rotation2d linearDirection = new Rotation2d(leftX, leftY);

    // Apply deadband
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, deadband);
    rightY = MathUtil.applyDeadband(rightY, deadband);

    // Apply squaring
    linearMagnitude =
        Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
    rightY = Math.copySign(rightY * rightY, rightY);

    // Calcaulate new linear components
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(
                GeomUtil.transformFromTranslation(linearMagnitude, 0.0))
            .getTranslation();

    // Send to drive
    double leftXMetersPerSec = linearVelocity.getX()
        * drive.getMaxLinearSpeedMetersPerSec() * speedLimitSupplier.get();
    double leftYMetersPerSec = linearVelocity.getY()
        * drive.getMaxLinearSpeedMetersPerSec() * speedLimitSupplier.get();
    double rightYRadPerSec =
        rightY * drive.getMaxAngularSpeedRadPerSec() * speedLimitSupplier.get();
    if (robotRelativeOverride.get()) {
      drive.runVelocity(new ChassisSpeeds(leftXMetersPerSec, leftYMetersPerSec,
          rightYRadPerSec));
    } else {
      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(leftXMetersPerSec,
          leftYMetersPerSec, rightYRadPerSec, drive.getRotation()));
    }
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
