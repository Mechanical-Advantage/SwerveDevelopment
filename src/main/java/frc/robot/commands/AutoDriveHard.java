// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;
import frc.robot.util.SuppliedCommand;
import frc.robot.util.trajectory.Waypoint;

public class AutoDriveHard extends SequentialCommandGroup {
  /** Creates a new AutoDriveHard. */
  public AutoDriveHard(Drive drive) {
    addCommands(new SuppliedCommand(() -> {
      Pose2d holonomicPose = drive.getPose();
      Translation2d fieldVelocity = drive.getFieldVelocity();
      boolean fieldVelocityIsZero = Math.abs(fieldVelocity.getX()) < 1e-3
          && Math.abs(fieldVelocity.getY()) < 1e-3;

      Waypoint start =
          fieldVelocityIsZero ? Waypoint.fromHolonomicPose(holonomicPose)
              : Waypoint.fromHolonomicPose(holonomicPose,
                  GeomUtil.direction(fieldVelocity));
      Waypoint end = Waypoint.fromHolonomicPose(RobotContainer.autoDriveTarget,
          RobotContainer.autoDriveTarget.getRotation());

      return new DriveTrajectory(drive, List.of(start, end),
          fieldVelocity.getNorm(), 0.0);
    }, drive));
  }
}
