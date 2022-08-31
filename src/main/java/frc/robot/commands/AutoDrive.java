// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.trajectory.CustomTrajectoryGenerator;
import frc.robot.util.trajectory.Waypoint;


public class AutoDrive extends CommandBase {
  private final double maxVelocityMetersPerSec;
  private final double maxAccelerationMetersPerSec2;
  private final double maxCentripetalAccelerationMetersPerSec2;

  private final RobotState robotState;
  private final Drive drive;
  private final Timer timer = new Timer();


  private final CustomTrajectoryGenerator customGenerator =
      new CustomTrajectoryGenerator();

  private static final Alert generatorAlert = new Alert(
      "Failed to generate all trajectories, check constants.", AlertType.ERROR);

  // Use addRequirements() here to declare subsystem dependencies.
  // Select max velocity and acceleration
  public AutoDrive(Drive drive, List<Waypoint> waypoints,
      RobotState robotState) {
    addRequirements(drive);
    this.drive = drive;
    this.robotState = robotState;

    boolean supportedRobot = true;
    switch (Constants.getRobot()) {
      case ROBOT_SIMBOT:
        maxVelocityMetersPerSec = Units.inchesToMeters(0.0);
        maxAccelerationMetersPerSec2 = Units.inchesToMeters(0.0);
        maxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(0.0);
        break;
      case ROBOT_2022S:
        maxVelocityMetersPerSec = Units.inchesToMeters(0.0);
        maxAccelerationMetersPerSec2 = Units.inchesToMeters(0.0);
        maxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(0.0);
        break;
      default:
        supportedRobot = false;
        maxVelocityMetersPerSec = 0.0;
        maxAccelerationMetersPerSec2 = 0.0;
        maxCentripetalAccelerationMetersPerSec2 = 0.0;
        break;
    }

    // setup trajectory configuration
    TrajectoryConfig config = new TrajectoryConfig(maxVelocityMetersPerSec,
        maxAccelerationMetersPerSec2)
            .setKinematics(
                new SwerveDriveKinematics(drive.getModuleTranslations()))
            .addConstraint(new CentripetalAccelerationConstraint(
                maxCentripetalAccelerationMetersPerSec2));

    // Generate trajectory
    try {
      customGenerator.generate(config, waypoints);
    } catch (TrajectoryGenerationException exception) {
      if (supportedRobot) {
        generatorAlert.set(true);
        DriverStation.reportError("Failed to generate trajectory.", true);
      }
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    State setpoint = Trajectory.sample(timer.get());
    Logger.getInstance().recordOutput("Odometry/ProfileSetpoint",
        new double[] {setpoint.poseMeters.getX(), setpoint.poseMeters.getY(),
            setpoint.poseMeters.getRotation().getRadians()});

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
