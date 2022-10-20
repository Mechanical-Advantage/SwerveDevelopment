// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotContainer;

public class AutoDriveSoft {
  private static final double convergeRatio = 0.5;
  private static final double turnKp = 5.0;
  private static final double driveMaxExp = 3.0;

  private static final Pose2d targetPose = RobotContainer.autoDriveTarget;
  private static final Rotation2d targetHolonomicRotation =
      RobotContainer.autoDriveTarget.getRotation();
  private static final double minRadius = 1.5;
  private static final double maxRadius = 3.0;

  private AutoDriveSoft() {}

  public static ChassisSpeeds calculate(ChassisSpeeds commandedSpeeds,
      Pose2d currentPose) {
    ChassisSpeeds speeds = new ChassisSpeeds(commandedSpeeds.vxMetersPerSecond,
        commandedSpeeds.vyMetersPerSecond,
        commandedSpeeds.omegaRadiansPerSecond);

    // Get control percent
    double controlPercent =
        (currentPose.getTranslation().getDistance(targetPose.getTranslation())
            - minRadius) / (maxRadius - minRadius);
    controlPercent = 1.0 - MathUtil.clamp(controlPercent, 0.0, 1.0);

    // Adjust turn speed
    Rotation2d holonomicRotationError =
        currentPose.getRotation().minus(targetHolonomicRotation);
    double maxTurnErrorRad = (1 - controlPercent) * Math.PI;
    if (holonomicRotationError.getRadians() < -maxTurnErrorRad) {
      speeds.omegaRadiansPerSecond +=
          +(-maxTurnErrorRad - holonomicRotationError.getRadians()) * turnKp;
    } else if (holonomicRotationError.getRadians() > maxTurnErrorRad) {
      speeds.omegaRadiansPerSecond +=
          +(maxTurnErrorRad - holonomicRotationError.getRadians()) * turnKp;
    }

    // Get intermediate target
    double xOffset = currentPose.relativeTo(targetPose).getX();
    Rotation2d targetDriveRotation;
    if (xOffset < 0.0) {
      Translation2d intermediateTarget = targetPose
          .transformBy(
              GeomUtil.transformFromTranslation(xOffset * convergeRatio, 0.0))
          .getTranslation();
      targetDriveRotation = GeomUtil
          .direction(intermediateTarget.minus(currentPose.getTranslation()));
    } else {
      targetDriveRotation = targetPose.getRotation();
    }

    // Get command direction and magnitude
    Rotation2d commandDirection =
        new Rotation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double commandMagnitude =
        Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    // Adjust drive direction
    Rotation2d commandDirectionRelative =
        commandDirection.minus(targetDriveRotation);
    boolean invertDirection =
        Math.abs(commandDirectionRelative.getRadians()) > Math.PI / 2.0;

    if (invertDirection) {
      commandDirectionRelative =
          commandDirectionRelative.rotateBy(Rotation2d.fromDegrees(180.0));
    }
    double directionNormalized =
        Math.abs(commandDirectionRelative.getRadians()) / (Math.PI / 2.0);
    directionNormalized = Math.pow(directionNormalized,
        Math.pow(2.0, controlPercent * driveMaxExp));
    commandDirectionRelative =
        new Rotation2d(Math.copySign(directionNormalized * (Math.PI / 2.0),
            commandDirectionRelative.getRadians()));
    if (invertDirection) {
      commandDirectionRelative =
          commandDirectionRelative.rotateBy(Rotation2d.fromDegrees(180.0));
    }

    Translation2d linearCommand = new Translation2d(commandMagnitude, 0.0)
        .rotateBy(targetDriveRotation.plus(commandDirectionRelative));
    speeds.vxMetersPerSecond = linearCommand.getX();
    speeds.vyMetersPerSecond = linearCommand.getY();

    return speeds;
  }
}
