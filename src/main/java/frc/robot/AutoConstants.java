// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.util.GeomUtil;

/** Useful constants copied from 2022 auto routines. */
public class AutoConstants {
  public static final Map<AutoPosition, Pose2d> cargoPositions =
      Map.of(AutoPosition.TARMAC_A,
          calcAimedPose(FieldConstants.cargoB
              .transformBy(GeomUtil.transformFromTranslation(-0.5, 0.0))),
          AutoPosition.TARMAC_C,
          calcAimedPose(FieldConstants.cargoD
              .transformBy(GeomUtil.transformFromTranslation(-0.5, 0.0))),
          AutoPosition.TARMAC_D, calcAimedPose(FieldConstants.cargoE
              .transformBy(GeomUtil.transformFromTranslation(-0.5, 0.0))));
  public static final Pose2d terminalCargoPosition =
      FieldConstants.cargoG.transformBy(new Transform2d(
          new Translation2d(0.35, 0.0), Rotation2d.fromDegrees(180.0)));
  public static final Pose2d terminalCargoApproachPosition =
      terminalCargoPosition
          .transformBy(GeomUtil.transformFromTranslation(-0.8, 0.0));

  public static Pose2d calcAimedPose(Pose2d pose) {
    Translation2d vehicleToCenter =
        FieldConstants.hubCenter.minus(pose.getTranslation());
    Rotation2d targetRotation =
        new Rotation2d(vehicleToCenter.getX(), vehicleToCenter.getY());
    targetRotation = targetRotation.plus(Rotation2d.fromDegrees(180));
    return new Pose2d(pose.getTranslation(), targetRotation);
  }
}
