// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.CubicHermiteSpline;
import edu.wpi.first.math.spline.Spline;
import frc.robot.FieldConstants;

public class AutoDriveSoftWithSpline {
  private static final Pose2d targetPose =
      FieldConstants.fenderA.transformBy(new Transform2d(
          new Translation2d(0.5, 0.0), Rotation2d.fromDegrees(180.0)));
  private static final Rotation2d targetHolonomicRotation =
      targetPose.getRotation();

  private static final double distanceTolerance = 0.1;
  private static final double turnMinRadius = 1.5;
  private static final double turnMaxRadius = 3.0;
  private static final double turnKp = 5.0;

  private AutoDriveSoftWithSpline() {}

  public static ChassisSpeeds calculate(Pose2d currentPose,
      double autoDriveSpeed, boolean adjustHolonomicRotation) {

    // Stop if close to target
    if (currentPose.getTranslation()
        .getDistance(targetPose.getTranslation()) < distanceTolerance) {
      autoDriveSpeed = 0.0;
    }

    // Generate spline
    var targetScalar = new Translation2d(1.5 * Math.min(2.0,
        targetPose.getTranslation().getDistance(currentPose.getTranslation())),
        0.0);
    var initialControl =
        new Spline.ControlVector(new double[] {currentPose.getX(), 0.0},
            new double[] {currentPose.getY(), 0.0});
    var finalControl = new Spline.ControlVector(
        new double[] {targetPose.getX(),
            targetScalar.rotateBy(targetPose.getRotation()).getX()},
        new double[] {targetPose.getY(),
            targetScalar.rotateBy(targetPose.getRotation()).getY()});
    var spline = new CubicHermiteSpline(initialControl.x, finalControl.x,
        initialControl.y, finalControl.y);
    var guideRotation = spline.getPoint(0.001).poseMeters.getRotation();

    // Log splines
    List<Pose2d> poses = new ArrayList<>();
    for (double t = 0.0; t < 1.0; t += 0.02) {
      poses.add(spline.getPoint(t).poseMeters);
    }
    poses.add(spline.getPoint(1.0).poseMeters);
    Logger.getInstance().recordOutput("Odometry/AutoDriveSpline",
        poses.toArray(Pose2d[]::new));

    // Calculate angular velocity
    double angularVelocity = 0.0;
    if (adjustHolonomicRotation) {
      double controlPercent =
          (currentPose.getTranslation().getDistance(targetPose.getTranslation())
              - turnMinRadius) / (turnMaxRadius - turnMinRadius);
      controlPercent = 1.0 - MathUtil.clamp(controlPercent, 0.0, 1.0);
      Rotation2d holonomicRotationError =
          currentPose.getRotation().minus(targetHolonomicRotation);
      double maxTurnErrorRad = (1 - controlPercent) * Math.PI;

      if (holonomicRotationError.getRadians() < -maxTurnErrorRad) {
        angularVelocity =
            (-maxTurnErrorRad - holonomicRotationError.getRadians()) * turnKp;
      } else if (holonomicRotationError.getRadians() > maxTurnErrorRad) {
        angularVelocity =
            (maxTurnErrorRad - holonomicRotationError.getRadians()) * turnKp;
      }
    }

    // Calculate chassis speeds
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        autoDriveSpeed * guideRotation.getCos(),
        autoDriveSpeed * guideRotation.getSin(), angularVelocity,
        currentPose.getRotation());
  }
}
