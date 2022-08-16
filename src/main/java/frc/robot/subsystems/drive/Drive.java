// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

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
  private Rotation2d[] turnPositionOffset = new Rotation2d[4];
  private Rotation2d[] turnPosition = new Rotation2d[4];
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

    // Calculate max angular speed (TODO: Find a better way to do this)
    SwerveModuleState[] states =
        kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 99999.0));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxLinearSpeed);
    maxAngularSpeed = kinematics.toChassisSpeeds(states).omegaRadiansPerSecond;
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

    // 2) On the first cycle, use the absolute encoders to reset the measured angles. You'll need to
    // record an offset value that's applied to future measurements. All of the calculations in
    // subsequent cycles should use the *relative* encoders.
    if (isFirstCycle) {
      isFirstCycle = false;
      for (int i = 0; i < 4; i++) {
        turnPositionOffset[i] =
            new Rotation2d(moduleInputs[i].turnAbsolutePositionRad);
      }
    }
    for (int i = 0; i < 4; i++) {
      turnPosition[i] = new Rotation2d(moduleInputs[i].turnPositionRad)
          .plus(turnPositionOffset[i]);
    }

    // 3) If running in normal closed loop mode (and the robot is enabled), run the kinematics. This
    // should include CALCULATING the module states, DESATURATING the wheel speeds, and OPTIMIZING
    // the module states (the SwerveDriveKinematics and SwerveModuleState classes have methods to do
    // all of that). Then, calculate the voltage commands for all four modules. The turn command
    // should use a PIDController (feedback control) and the drive command should use both a
    // SimpleMotorFeedforward and PIDController (feedforward and feedback control). Check the
    // documentation for those classes to help you.
    //
    // Also - remember the UNITS. Once you calculate the module states, convert from meters/sec to
    // radians/sec and run the feedforward and feedback control in radians.
    if (isNormalClosedLoopMode && DriverStation.isEnabled()) {
      SwerveModuleState[] moduleStates =
          kinematics.toSwerveModuleStates(closedLoopSetpoint);
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxLinearSpeed);
      for (int i = 0; i < 4; i++) {
        SwerveModuleState optimizedState =
            SwerveModuleState.optimize(moduleStates[i], turnPosition[i]);
        moduleIOs[i].setTurnVoltage(turnFeedback[i].calculate(
            turnPosition[i].getRadians(), optimizedState.angle.getRadians()));

        double velocityRadPerSec =
            optimizedState.speedMetersPerSecond / wheelRadius;
        moduleIOs[i]
            .setDriveVoltage(driveFeedforward.calculate(velocityRadPerSec)
                + driveFeedback[i].calculate(
                    moduleInputs[i].driveVelocityRadPerSec, velocityRadPerSec));
      }
    }

    // 4) If running in characterization mode (and the robot is enabled), command the requested
    // voltage to the drive motors and run the PID controller to bring the angle to zero degrees.
    if (!isNormalClosedLoopMode && DriverStation.isEnabled()) {
      for (int i = 0; i < 4; i++) {
        moduleIOs[i].setTurnVoltage(
            turnFeedback[i].calculate(turnPosition[i].getRadians(), 0.0));
        moduleIOs[i].setDriveVoltage(characterizationVoltage);
      }
    }

    if (DriverStation.isDisabled()) {
      for (int i = 0; i < 4; i++) {
        moduleIOs[i].setTurnVoltage(0.0);
        moduleIOs[i].setDriveVoltage(0.0);
      }
    }

    // 5) Update the odometry pose based on the module velocities. If the gyro is connected, use it
    // to find the angle difference. If the gyro is disconnected, estimate the angle differene based
    // on the omega componenet of the ChassisSpeeds object. The link below should give you a good
    // starting point.
    // https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/kinematics/SwerveDriveOdometry.java#L100-L106
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      measuredStates[i] = new SwerveModuleState(
          moduleInputs[i].driveVelocityRadPerSec * wheelRadius,
          turnPosition[i]);
    }
    ChassisSpeeds chassisState = kinematics.toChassisSpeeds(measuredStates);
    if (gyroInputs.connected) {
      odometryPose = odometryPose.exp(
          new Twist2d(chassisState.vxMetersPerSecond * Constants.loopPeriodSecs,
              chassisState.vyMetersPerSecond * Constants.loopPeriodSecs,
              gyroInputs.positionRad - lastGyroPosRad));
    } else {
      odometryPose = odometryPose.exp(
          new Twist2d(chassisState.vxMetersPerSecond * Constants.loopPeriodSecs,
              chassisState.vyMetersPerSecond * Constants.loopPeriodSecs,
              chassisState.omegaRadiansPerSecond * Constants.loopPeriodSecs));
    }
    lastGyroPosRad = gyroInputs.positionRad;


    // 6) Enable or disable brake mode on the turn and drive motors. They should be in brake model
    // when the robot is enabled, and switch to coast when the robot is disabled *and it comes to a
    // stop*. See the example below from our 2022 code. Also discuss - why is this useful?
    // https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/subsystems/drive/Drive.java#L206-L221
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
