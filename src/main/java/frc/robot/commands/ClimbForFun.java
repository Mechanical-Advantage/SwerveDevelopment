// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;
import frc.robot.util.trajectory.Waypoint;

public class ClimbForFun extends SequentialCommandGroup {
  public static final Pose2d extendPose = new Pose2d(
      new Translation2d(FieldConstants.hangarLength - 1.2,
          FieldConstants.fieldWidth - (FieldConstants.hangarWidth / 2)),
      Rotation2d.fromDegrees(180.0));
  public static final double climbExtendPercent = 1.0;
  public static final double climbExtendTime = 2.0;
  public static final Pose2d grabPose =
      extendPose.transformBy(GeomUtil.transformFromTranslation(-0.5, 0.0));
  public static final double grabVelocity = Units.inchesToMeters(10.0);
  public static final double climbRetractPercent = -1.0;
  public static final double climbRetractTime = 1.0;

  /** Creates a new ClimbForFun. */
  public ClimbForFun(Drive drive, Climber climber) {
    addCommands(
        new AutoDrive(drive,
            List.of(Waypoint.fromHolonomicPose(AutoPosition.TARMAC_A.getPose()),
                Waypoint.fromHolonomicPose(extendPose,
                    extendPose.getRotation()))),
        new StartEndCommand(() -> climber.runPercent(climbExtendPercent),
            climber::stop, climber).withTimeout(climbExtendTime),
        new AutoDrive(drive,
            List.of(Waypoint.fromHolonomicPose(extendPose),
                Waypoint.fromHolonomicPose(grabPose)),
            List.of(new MaxVelocityConstraint(grabVelocity))),
        new StartEndCommand(() -> climber.runPercent(climbRetractPercent),
            climber::stop, climber).withTimeout(climbRetractTime));
  }
}
