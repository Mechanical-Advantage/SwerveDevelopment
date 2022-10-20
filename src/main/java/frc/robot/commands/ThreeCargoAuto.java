// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// hello

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.trajectory.Waypoint;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeCargoAuto extends SequentialCommandGroup {

  public static Pose2d startPosition = AutoPosition.TARMAC_D.getPose();
  public static Pose2d cargoEPosition = FieldConstants.cargoE.transformBy(
      new Transform2d(new Translation2d(-0.5, 0), Rotation2d.fromDegrees(0.0)));
  public static Pose2d cargoDPosition = calcAimedPose(FieldConstants.cargoD);
  public static Pose2d halfCargoDPosition =
      FieldConstants.cargoD.transformBy(new Transform2d(
          new Translation2d(0.2, 0.2), Rotation2d.fromDegrees(-75)));

  /** Creates a new ThreeCargoAuto. */
  public ThreeCargoAuto(Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new DriveTrajectory(drive,
            List.of(Waypoint.fromHolonomicPose(startPosition),
                Waypoint.fromHolonomicPose(cargoEPosition))),
        new DriveTrajectory(drive,
            List.of(Waypoint.fromHolonomicPose(cargoEPosition),
                Waypoint.fromHolonomicPose(halfCargoDPosition),
                Waypoint.fromHolonomicPose(cargoDPosition))));
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
