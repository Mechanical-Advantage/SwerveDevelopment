// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// balls

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.trajectory.Waypoint;

public class FiveCargoAuto extends SequentialCommandGroup {

  public static Pose2d cargoGPosition = FieldConstants.cargoG.transformBy(
      new Transform2d(new Translation2d(0.5, 0), Rotation2d.fromDegrees(175)));
  public static Pose2d endPosition = calcAimedPose(FieldConstants.cargoD);

  /** Creates a new FiveCargoAuto. */
  public FiveCargoAuto(Drive drive) {
    addCommands(new ThreeCargoAuto(drive),
        new DriveTrajectory(drive,
            List.of(Waypoint.fromHolonomicPose(ThreeCargoAuto.cargoDPosition),
                Waypoint.fromHolonomicPose(cargoGPosition))),
        new DriveTrajectory(drive,
            List.of(Waypoint.fromHolonomicPose(cargoGPosition),
                Waypoint.fromHolonomicPose(endPosition))));

  }

  public static Pose2d calcAimedPose(Pose2d pose) {
    Translation2d vehicleToCenter =
        FieldConstants.hubCenter.minus(pose.getTranslation());
    Rotation2d targetRotation =
        new Rotation2d(vehicleToCenter.getX(), vehicleToCenter.getY());
    targetRotation = targetRotation.plus(Rotation2d.fromDegrees(180));
    return new Pose2d(pose.getTranslation(), targetRotation);
  }
}


