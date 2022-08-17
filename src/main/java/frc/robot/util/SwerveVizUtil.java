// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Utility class to render swerve modules with the Advantage Scope point visualizer. */
public class SwerveVizUtil {
  /**
   * Logs a visualization of swerve module states for the Advantage Scope point visualizer. The
   * output arrays should be rendered at 1000x1000 pixel resolution.
   * 
   * @param key The name of the log field ("X" and "Y" will be appended)
   * @param states The swerve module states to log (FL, FR, BL, BR)
   * @param maxSpeed The maximum speed of the swerve modules
   */
  public static void logModuleStates(String key, SwerveModuleState[] states,
      double maxSpeed) {
    final double maxLengthPx = 150.0;
    final List<Double> xPoints = new ArrayList<>();
    final List<Double> yPoints = new ArrayList<>();
    final List<Double> centerXs = List.of(1000.0 * (1.0 / 3.0),
        1000.0 * (2.0 / 3.0), 1000.0 * (1.0 / 3.0), 1000.0 * (2.0 / 3.0));
    final List<Double> centerYs = List.of(1000.0 * (1.0 / 3.0),
        1000.0 * (2.0 / 3.0), 1000.0 * (1.0 / 3.0), 1000.0 * (2.0 / 3.0));

    for (int i = 0; i < 4; i++) {
      Translation2d scaledTranslation =
          new Pose2d(new Translation2d(), states[i].angle)
              .transformBy(GeomUtil.transformFromTranslation(
                  (states[i].speedMetersPerSecond / maxSpeed) * maxLengthPx,
                  0.0))
              .getTranslation();

      xPoints.add(centerXs.get(i));
      yPoints.add(centerYs.get(i));
      xPoints.add(centerXs.get(i) - scaledTranslation.times(0.25).getY());
      yPoints.add(centerYs.get(i) - scaledTranslation.times(0.25).getX());
      xPoints.add(centerXs.get(i) - scaledTranslation.times(0.50).getY());
      yPoints.add(centerYs.get(i) - scaledTranslation.times(0.50).getX());
      xPoints.add(centerXs.get(i) - scaledTranslation.times(0.75).getY());
      yPoints.add(centerYs.get(i) - scaledTranslation.times(0.75).getX());
      xPoints.add(centerXs.get(i) - scaledTranslation.getY());
      yPoints.add(centerYs.get(i) - scaledTranslation.getX());

      Logger.getInstance().recordOutput(key + "X",
          xPoints.stream().mapToDouble(Double::doubleValue).toArray());
      Logger.getInstance().recordOutput(key + "Y",
          yPoints.stream().mapToDouble(Double::doubleValue).toArray());
    }
  }
}
