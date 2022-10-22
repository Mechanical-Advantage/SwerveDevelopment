// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;
import frc.robot.util.trajectory.Waypoint;

public class ScoreWithForce extends SequentialCommandGroup {

  /** Creates a new ScoreWithForce. */
  public ScoreWithForce(Drive drive, AutoPosition startPosition) {
    Pose2d recoilPosition = startPosition.getPose()
        .transformBy(GeomUtil.transformFromTranslation(-1.2, 0.0));
    Pose2d crashPosition = startPosition.getPose()
        .transformBy(GeomUtil.transformFromTranslation(0.5, 0.0));
    Pose2d taxiPosition = startPosition.getPose().transformBy(new Transform2d(
        new Translation2d(-2.5, 0.0), Rotation2d.fromDegrees(180.0)));

    addCommands(new WaitCommand(8.0),
        new DriveTrajectory(drive,
            List.of(Waypoint.fromHolonomicPose(startPosition.getPose()),
                Waypoint.fromHolonomicPose(recoilPosition))),
        new DriveTrajectory(drive,
            List.of(Waypoint.fromHolonomicPose(recoilPosition),
                Waypoint.fromHolonomicPose(crashPosition)),
            0.0, Double.POSITIVE_INFINITY),
        new WaitCommand(2.0),
        new DriveTrajectory(drive,
            List.of(Waypoint.fromHolonomicPose(crashPosition),
                Waypoint.fromHolonomicPose(taxiPosition))));
  }
}
