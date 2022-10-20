// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoDriveSoft;
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
  private final Supplier<String> modeSupplier;
  private final Supplier<Double> linearSpeedLimitSupplier;
  private final Supplier<Double> angularSpeedLimitSupplier;
  private final Supplier<Boolean> autoDriveSupplier;

  private static final double deadband = 0.1;

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(Drive drive, Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier, Supplier<Double> rightYSupplier,
      Supplier<Boolean> robotRelativeOverride, Supplier<String> modeSupplier,
      Supplier<Double> linearSpeedLimitSupplier,
      Supplier<Double> angularSpeedLimitSupplier,
      Supplier<Boolean> autoDriveSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightYSupplier = rightYSupplier;
    this.robotRelativeOverride = robotRelativeOverride;
    this.modeSupplier = modeSupplier;
    this.linearSpeedLimitSupplier = linearSpeedLimitSupplier;
    this.angularSpeedLimitSupplier = angularSpeedLimitSupplier;
    this.autoDriveSupplier = autoDriveSupplier;
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

    // Apply speed limits
    linearMagnitude *= linearSpeedLimitSupplier.get();
    rightY *= angularSpeedLimitSupplier.get();

    // Calcaulate new linear components
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(
                GeomUtil.transformFromTranslation(linearMagnitude, 0.0))
            .getTranslation();
    if (modeSupplier.get() == "Tank") {
      linearVelocity = new Translation2d(linearVelocity.getX(), 0.0);
    }

    // Send to drive
    double leftXMetersPerSec =
        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec();
    double leftYMetersPerSec =
        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec();
    double rightYRadPerSec = rightY * drive.getMaxAngularSpeedRadPerSec();

    if (robotRelativeOverride.get() || modeSupplier.get() == "Tank") {
      drive.runVelocity(new ChassisSpeeds(leftXMetersPerSec, leftYMetersPerSec,
          rightYRadPerSec));
    } else {
      ChassisSpeeds commandedSpeeds = new ChassisSpeeds(leftXMetersPerSec,
          leftYMetersPerSec, rightYRadPerSec);
      ChassisSpeeds adjustedSpeeds = autoDriveSupplier.get()
          ? AutoDriveSoft.calculate(commandedSpeeds, drive.getPose())
          : commandedSpeeds;
      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
          adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond,
          adjustedSpeeds.omegaRadiansPerSecond, drive.getRotation()));
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
