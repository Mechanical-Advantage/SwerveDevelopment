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
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.spline.SplineHelper;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants2023;

public class AutoDriveSoftWithSpline {
  private static final Pose2d targetPose =
      new Pose2d(FieldConstants2023.Grids.lowTranslations[2]
          .plus(new Translation2d(0.75, 0.0)), Rotation2d.fromDegrees(180.0));
  private static final Rotation2d targetHolonomicRotation =
      Rotation2d.fromDegrees(180.0);

  private static final double distanceTolerance = 0.1;
  private static final double turnMinRadius = 1.0;
  private static final double turnMaxRadius = 3.0;
  private static final double turnKp = 5.0;

  private static final List<Pose2d> fullSplinePoints;

  private AutoDriveSoftWithSpline() {}

  static {
    List<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(FieldConstants2023.LoadingZone.innerX,
        (FieldConstants2023.LoadingZone.rightY
            + FieldConstants2023.LoadingZone.leftY) / 2.0,
        Rotation2d.fromDegrees(180.0)));
    waypoints.add(new Pose2d(FieldConstants2023.LoadingZone.midX,
        (FieldConstants2023.LoadingZone.rightY
            + FieldConstants2023.LoadingZone.leftY) / 2.0,
        Rotation2d.fromDegrees(180.0)));
    waypoints.add(new Pose2d(FieldConstants2023.Community.midX - 0.5,
        (FieldConstants2023.Community.leftY + FieldConstants2023.Community.midY)
            / 2.0 - 0.2,
        Rotation2d.fromDegrees(180.0)));

    var endTranslation = FieldConstants2023.Grids.lowTranslations[2]
        .plus(new Translation2d(0.75, 0.0));
    var endRotation = new Translation2d(FieldConstants2023.Community.midX,
        (FieldConstants2023.Community.leftY + FieldConstants2023.Community.midY)
            / 2.0).minus(endTranslation).getAngle()
                .plus(Rotation2d.fromDegrees(180.0));
    waypoints.add(new Pose2d(endTranslation, endRotation));

    List<Pose2d> splinePoints = new ArrayList<>();
    splinePoints.addAll(TrajectoryGenerator
        .splinePointsFromSplines(
            SplineHelper.getQuinticSplinesFromWaypoints(waypoints))
        .stream().map((PoseWithCurvature pose) -> pose.poseMeters).toList());
    int i = 0;
    while (i < splinePoints.size() - 1) {
      Translation2d lastTranslation = splinePoints.get(i).getTranslation();
      i++;
      while (i < splinePoints.size() - 1 && splinePoints.get(i).getTranslation()
          .getDistance(lastTranslation) < 0.25) {
        splinePoints.remove(i);
      }
    }
    fullSplinePoints = splinePoints;
  }

  public static ChassisSpeeds calculate(Pose2d currentPose,
      double autoDriveSpeed, boolean adjustHolonomicRotation) {

    // Stop if close to target
    if (currentPose.getTranslation()
        .getDistance(targetPose.getTranslation()) < distanceTolerance) {
      autoDriveSpeed = 0.0;
    }

    // Get intermediate target
    Logger.getInstance().recordOutput("AutoDrive2023/FullSpline",
        fullSplinePoints.toArray(new Pose2d[fullSplinePoints.size()]));
    double minDistance = Double.POSITIVE_INFINITY;
    int minDistanceIndex = 0;
    for (int i = 0; i < fullSplinePoints.size(); i++) {
      var distance = fullSplinePoints.get(i).getTranslation()
          .getDistance(currentPose.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        minDistanceIndex = i;
      }
    }
    Pose2d intermediateTarget = fullSplinePoints
        .get(Math.min(minDistanceIndex + 4, fullSplinePoints.size() - 1));
    Logger.getInstance().recordOutput("AutoDrive2023/ClosestPose",
        fullSplinePoints.get(minDistanceIndex));
    Logger.getInstance().recordOutput("AutoDrive2023/IntermediateTarget",
        intermediateTarget);

    // Generate spline
    var targetScalar = new Translation2d(1.5 * intermediateTarget
        .getTranslation().getDistance(currentPose.getTranslation()), 0.0);
    var initialControl =
        new Spline.ControlVector(new double[] {currentPose.getX(), 0.0},
            new double[] {currentPose.getY(), 0.0});
    var finalControl = new Spline.ControlVector(
        new double[] {intermediateTarget.getX(),
            targetScalar.rotateBy(intermediateTarget.getRotation()).getX()},
        new double[] {intermediateTarget.getY(),
            targetScalar.rotateBy(intermediateTarget.getRotation()).getY()});
    var spline = new CubicHermiteSpline(initialControl.x, finalControl.x,
        initialControl.y, finalControl.y);
    var guideRotation = spline.getPoint(0.001).poseMeters.getRotation();

    // Log partial spline
    List<Pose2d> poses = new ArrayList<>();
    for (double t = 0.0; t < 1.0; t += 0.02) {
      poses.add(spline.getPoint(t).poseMeters);
    }
    poses.add(spline.getPoint(1.0).poseMeters);
    Logger.getInstance().recordOutput("AutoDrive2023/PartialSpline",
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
