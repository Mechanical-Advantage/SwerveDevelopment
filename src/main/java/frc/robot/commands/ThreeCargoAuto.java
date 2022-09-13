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
import frc.robot.FieldConstants;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.trajectory.Waypoint;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeCargoAuto extends SequentialCommandGroup {

  public static AutoPosition startPosition = AutoPosition.TARMAC_D;
  public static Pose2d cargoEPosition =
      FieldConstants.referenceD.transformBy(new Transform2d(
          new Translation2d(-1.0, -1.0), Rotation2d.fromDegrees(0.0)));
  public static Pose2d cargoDPosition =
      FieldConstants.referenceC.transformBy(new Transform2d(
          new Translation2d(-1.0, null), Rotation2d.fromDegrees(90.0)));

  /** Creates a new ThreeCargoAuto. */
  public ThreeCargoAuto(Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
    new AutoDrive(drive, List.of(
        new Waypoint(new Translation2d(), new Rotation2d(), new Rotation2d()),
        new Waypoint(new Translation2d())));
  }
}
