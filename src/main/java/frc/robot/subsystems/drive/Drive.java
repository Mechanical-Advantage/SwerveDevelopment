// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;
import frc.robot.util.SwerveVizUtil;
import frc.robot.util.TunableNumber;

public class Drive extends SubsystemBase {
  private static final double maxCoastVelocityMetersPerSec = 0.05; // Need to be under this to
                                                                   // switch to coast when disabling

  private final GyroIO gyroIO;
  private final GyroIOInputs gyroInputs = new GyroIOInputs();
  private final ModuleIO[] moduleIOs = new ModuleIO[4]; // FL, FR, BL, BR
  private final ModuleIOInputs[] moduleInputs =
      new ModuleIOInputs[] {new ModuleIOInputs(), new ModuleIOInputs(),
          new ModuleIOInputs(), new ModuleIOInputs()};

  private final double maxLinearSpeed;
  private final double maxAngularSpeed;
  private final double wheelRadius;
  private final double trackWidthX;
  private final double trackWidthY;

  private final TunableNumber driveKp = new TunableNumber("Drive/DriveKp");
  private final TunableNumber driveKd = new TunableNumber("Drive/DriveKd");
  private final TunableNumber driveKs = new TunableNumber("Drive/DriveKs");
  private final TunableNumber driveKv = new TunableNumber("Drive/DriveKv");

  private final TunableNumber turnKp = new TunableNumber("Drive/TurnKp");
  private final TunableNumber turnKd = new TunableNumber("Drive/TurnKd");

  private final SwerveDriveKinematics kinematics;
  private SimpleMotorFeedforward driveFeedforward;
  private final PIDController[] driveFeedback = new PIDController[4];
  private final PIDController[] turnFeedback = new PIDController[4];

  private boolean isFirstCycle = true;
  private Rotation2d[] turnPositionOffsets = new Rotation2d[4];
  private Rotation2d[] turnPositions = new Rotation2d[4];
  private Pose2d odometryPose = new Pose2d();
  private double lastGyroPosRad = 0.0;
  private boolean brakeMode = false;

  private boolean isNormalClosedLoopMode = false;
  private ChassisSpeeds closedLoopSetpoint = new ChassisSpeeds();
  private double characterizationVoltage = 0.0;

  /** Creates a new Drive. */
  public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO,
      ModuleIO blModuleIO, ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    moduleIOs[0] = flModuleIO;
    moduleIOs[1] = frModuleIO;
    moduleIOs[2] = blModuleIO;
    moduleIOs[3] = brModuleIO;

    switch (Constants.getRobot()) {
      case ROBOT_SIMBOT:
        maxLinearSpeed = Units.feetToMeters(14.5);
        wheelRadius = Units.inchesToMeters(2.0);
        trackWidthX = 0.65;
        trackWidthY = 0.65;

        driveKp.setDefault(0.0);
        driveKd.setDefault(0.0);
        driveKs.setDefault(0.0);
        driveKv.setDefault(0.0);

        turnKp.setDefault(0.0);
        turnKd.setDefault(0.0);
        break;
      default:
        maxLinearSpeed = 0.0;
        wheelRadius = 0.0;
        trackWidthX = 0.0;
        trackWidthY = 0.0;

        driveKp.setDefault(0.0);
        driveKd.setDefault(0.0);
        driveKs.setDefault(0.0);
        driveKv.setDefault(0.0);

        turnKp.setDefault(0.0);
        turnKd.setDefault(0.0);
        break;
    }

    kinematics = new SwerveDriveKinematics(getModuleTranslations());
    driveFeedforward = new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
    for (int i = 0; i < 4; i++) {
      driveFeedback[i] = new PIDController(driveKp.get(), 0.0, driveKd.get(),
          Constants.loopPeriodSecs);
      turnFeedback[i] = new PIDController(turnKp.get(), 0.0, turnKd.get(),
          Constants.loopPeriodSecs);
      turnFeedback[i].enableContinuousInput(-Math.PI, Math.PI);
    }

    // Calculate max angular speed
    maxAngularSpeed = maxLinearSpeed / Arrays.stream(getModuleTranslations())
        .map(translation -> translation.getNorm()).max(Double::compare).get();
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
    for (int i = 0; i < 4; i++) {
      moduleIOs[i].updateInputs(moduleInputs[i]);
      Logger.getInstance().processInputs("Drive/Module" + Integer.toString(i),
          moduleInputs[i]);
    }

    // Update objects based on TunableNumbers
    if (driveKp.hasChanged() || driveKd.hasChanged() || driveKs.hasChanged()
        || driveKv.hasChanged() || turnKp.hasChanged() || turnKd.hasChanged()) {
      driveFeedforward =
          new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
      for (int i = 0; i < 4; i++) {
        driveFeedback[i].setP(driveKp.get());
        driveFeedback[i].setD(driveKd.get());
        turnFeedback[i].setP(turnKp.get());
        turnFeedback[i].setD(turnKd.get());
      }
    }

    // Update angle measurements (resets from absolute encoders on first cycle)
    if (isFirstCycle) {
      isFirstCycle = false;
      for (int i = 0; i < 4; i++) {
        turnPositionOffsets[i] =
            new Rotation2d(moduleInputs[i].turnAbsolutePositionRad);
      }
    }
    for (int i = 0; i < 4; i++) {
      turnPositions[i] = new Rotation2d(moduleInputs[i].turnPositionRad)
          .plus(turnPositionOffsets[i]);
    }

    // In normal mode, run the controllers for turning and driving based on the current setpoint
    if (isNormalClosedLoopMode && DriverStation.isEnabled()) {
      SwerveModuleState[] moduleStates =
          kinematics.toSwerveModuleStates(closedLoopSetpoint);
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxLinearSpeed);
      SwerveVizUtil.logModuleStates("SetpointStates", moduleStates,
          maxLinearSpeed);
      for (int i = 0; i < 4; i++) {
        SwerveModuleState optimizedState =
            SwerveModuleState.optimize(moduleStates[i], turnPositions[i]);
        moduleIOs[i].setTurnVoltage(turnFeedback[i].calculate(
            turnPositions[i].getRadians(), optimizedState.angle.getRadians()));

        double velocityRadPerSec =
            optimizedState.speedMetersPerSecond / wheelRadius;
        moduleIOs[i]
            .setDriveVoltage(driveFeedforward.calculate(velocityRadPerSec)
                + driveFeedback[i].calculate(
                    moduleInputs[i].driveVelocityRadPerSec, velocityRadPerSec));
      }
    }

    // In characterization mode, drive at the specified voltage (and turn to zero degrees)
    if (!isNormalClosedLoopMode && DriverStation.isEnabled()) {
      for (int i = 0; i < 4; i++) {
        moduleIOs[i].setTurnVoltage(
            turnFeedback[i].calculate(turnPositions[i].getRadians(), 0.0));
        moduleIOs[i].setDriveVoltage(characterizationVoltage);
      }
    }

    // Disable output while disabled
    if (DriverStation.isDisabled()) {
      for (int i = 0; i < 4; i++) {
        moduleIOs[i].setTurnVoltage(0.0);
        moduleIOs[i].setDriveVoltage(0.0);
      }
    }

    // Update odometry
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      measuredStates[i] = new SwerveModuleState(
          moduleInputs[i].driveVelocityRadPerSec * wheelRadius,
          turnPositions[i]);
    }
    ChassisSpeeds chassisState = kinematics.toChassisSpeeds(measuredStates);
    if (gyroInputs.connected) { // Use gyro for angular change when connected
      odometryPose = odometryPose.exp(
          new Twist2d(chassisState.vxMetersPerSecond * Constants.loopPeriodSecs,
              chassisState.vyMetersPerSecond * Constants.loopPeriodSecs,
              gyroInputs.positionRad - lastGyroPosRad));
    } else { // Fall back to using angular velocity (disconnected or sim)
      odometryPose = odometryPose.exp(
          new Twist2d(chassisState.vxMetersPerSecond * Constants.loopPeriodSecs,
              chassisState.vyMetersPerSecond * Constants.loopPeriodSecs,
              chassisState.omegaRadiansPerSecond * Constants.loopPeriodSecs));
    }
    lastGyroPosRad = gyroInputs.positionRad;
    Logger.getInstance().recordOutput("Odometry",
        new double[] {odometryPose.getX(), odometryPose.getY(),
            odometryPose.getRotation().getRadians()});


    // Enable/disable brake mode
    if (DriverStation.isEnabled()) {
      if (!brakeMode) {
        brakeMode = true;
        for (int i = 0; i < 4; i++) {
          moduleIOs[i].setTurnBrakeMode(true);
          moduleIOs[i].setDriveBrakeMode(true);
        }
      }
    } else {
      boolean stillMoving = false;
      for (int i = 0; i < 4; i++) {
        if (Math.abs(moduleInputs[i].driveVelocityRadPerSec
            * wheelRadius) > maxCoastVelocityMetersPerSec) {
          stillMoving = true;
        }
      }

      if (brakeMode && !stillMoving) {
        brakeMode = false;
        for (int i = 0; i < 4; i++) {
          moduleIOs[i].setTurnBrakeMode(false);
          moduleIOs[i].setDriveBrakeMode(false);
        }
      }
    }
  }

  /**
   * Runs the drive at the desired velocity.
   * 
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    isNormalClosedLoopMode = true;
    closedLoopSetpoint = speeds;
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxLinearSpeed;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxAngularSpeed;
  }

  /** Returns the current odometry pose. */
  public Pose2d getPose() {
    return odometryPose;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return odometryPose.getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    odometryPose = pose;
  }

  /** Returns an array of module translations. */
  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
        new Translation2d(trackWidthX / 2.0, trackWidthY / 2.0),
        new Translation2d(trackWidthX / 2.0, -trackWidthY / 2.0),
        new Translation2d(-trackWidthX / 2.0, trackWidthY / 2.0),
        new Translation2d(-trackWidthX / 2.0, -trackWidthY / 2.0)};
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    isNormalClosedLoopMode = false;
    characterizationVoltage = volts;
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (int i = 0; i < 4; i++) {
      driveVelocityAverage += moduleInputs[i].driveVelocityRadPerSec;
    }
    return driveVelocityAverage / 4.0;
  }
}
