// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.MathUtil;
import java.util.function.Supplier;

public class DriveWithJoysticks extends CommandBase {

  private final Drive drive;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightYSupplier;

  private static final double deadband = 0.1;

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(Drive drive, Supplier<Double> leftXSupplier, Supplier<Double> leftYSupplier, Supplier<Double> rightYSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightYSupplier = rightYSupplier;
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
    
    // Apply deadband
    leftX = MathUtil.applyDeadband(leftX, deadband);
    leftY = MathUtil.applyDeadband(leftY, deadband);
    rightY = MathUtil.applyDeadband(rightY, deadband);

    // Apply squaring
    leftX = Math.copySign(leftX * leftX, leftX);
    leftY = Math.copySign(leftY * leftY, leftY);
    rightY = Math.copySign(rightY * rightY, rightY);
    
    // Send to drive
    double leftXMetersPerSec = leftX * drive.getMaxLinearSpeedMetersPerSec();
    double leftYMetersPerSec = leftY * drive.getMaxLinearSpeedMetersPerSec();
    double rightYRadPerSec = rightY * drive.getMaxAngularSpeedRadPerSec();
    drive.runVelocity(new ChassisSpeeds(leftXMetersPerSec, leftYMetersPerSec, rightYRadPerSec)); 
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
