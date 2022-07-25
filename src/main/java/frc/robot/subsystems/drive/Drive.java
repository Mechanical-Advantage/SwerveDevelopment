// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;

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
import frc.robot.util.GeomUtil;
import frc.robot.util.TunableNumber;

public class Drive extends SubsystemBase {
  private final GyroIO gyroIO;
  private final GyroIOInputs gyroInputs = new GyroIOInputs();
  private final ModuleIO[] moduleIOs = new ModuleIO[4];
  private final ModuleIOInputs[] moduleInputs = new ModuleIOInputs[4];

  private final TunableNumber maxSpeed = new TunableNumber("Drive/MaxSpeed");
  private final TunableNumber wheelRadius =
      new TunableNumber("Drive/WheelRadius");
  private final TunableNumber trackWidthX =
      new TunableNumber("Drive/TrackWidthX");
  private final TunableNumber trackWidthY =
      new TunableNumber("Drive/TrackWidthY");

  private final TunableNumber driveKs = new TunableNumber("Drive/DriveKS");
  private final TunableNumber driveKv = new TunableNumber("Drive/DriveKV");
  private final TunableNumber driveKp = new TunableNumber("Drive/DriveKP");
  private final TunableNumber driveKd = new TunableNumber("Drive/DriveKD");
  private final TunableNumber turnKp = new TunableNumber("Drive/TurnKP");
  private final TunableNumber turnKd = new TunableNumber("Drive/TurnKD");

  private SwerveDriveKinematics kinematics;
  private SimpleMotorFeedforward driveFeedForward;
  private PIDController[] driveControllers = new PIDController[4];
  private PIDController[] turnControllers = new PIDController[4];

  private ChassisSpeeds setpointSpeeds = new ChassisSpeeds();
  private Double characterizationVolts = null;
  private boolean firstCycle = true;
  private double lastYawPositionRad = 0.0;
  private Pose2d pose = new Pose2d();

  /** Creates a new Drive. */
  public Drive(GyroIO gyroIO, ModuleIO frontLeftModuleIO,
      ModuleIO frontRightModuleIO, ModuleIO backLeftModuleIO,
      ModuleIO backRightModuleIO) {
    this.gyroIO = gyroIO;
    this.moduleIOs[0] = frontLeftModuleIO;
    this.moduleIOs[1] = frontRightModuleIO;
    this.moduleIOs[2] = backLeftModuleIO;
    this.moduleIOs[3] = backRightModuleIO;
    for (int i = 0; i < 4; i++) {
      moduleInputs[i] = new ModuleIOInputs();
      moduleIOs[i].setDriveBrakeMode(true);
      moduleIOs[i].setTurnBrakeMode(true);

      driveControllers[i] =
          new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
      turnControllers[i] =
          new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
      turnControllers[i].enableContinuousInput(-Math.PI, Math.PI);
    }

    switch (Constants.getRobot()) {
      case ROBOT_SIMBOT:
        maxSpeed.setDefault(Units.inchesToMeters(179.0));
        wheelRadius.setDefault(Units.inchesToMeters(2.0));
        trackWidthX.setDefault(0.65);
        trackWidthY.setDefault(0.65);
        driveKs.setDefault(0.00262);
        driveKv.setDefault(0.13394);
        driveKp.setDefault(0.5);
        driveKd.setDefault(0.0);
        turnKp.setDefault(10.0);
        turnKd.setDefault(0.0);
        break;
      default:
        maxSpeed.setDefault(Units.inchesToMeters(100.0));
        wheelRadius.setDefault(Units.inchesToMeters(1.0));
        trackWidthX.setDefault(1.0);
        trackWidthY.setDefault(1.0);
        driveKs.setDefault(0.0);
        driveKv.setDefault(0.0);
        driveKp.setDefault(0.0);
        driveKd.setDefault(0.0);
        turnKp.setDefault(0.0);
        turnKd.setDefault(0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    moduleIOs[0].updateInputs(moduleInputs[0]);
    moduleIOs[1].updateInputs(moduleInputs[1]);
    moduleIOs[2].updateInputs(moduleInputs[2]);
    moduleIOs[3].updateInputs(moduleInputs[3]);
    Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
    Logger.getInstance().processInputs("Drive/Module0", moduleInputs[0]);
    Logger.getInstance().processInputs("Drive/Module1", moduleInputs[1]);
    Logger.getInstance().processInputs("Drive/Module2", moduleInputs[2]);
    Logger.getInstance().processInputs("Drive/Module3", moduleInputs[3]);

    // Update kinematics
    if (firstCycle || trackWidthX.hasChanged() || trackWidthY.hasChanged()) {
      kinematics = new SwerveDriveKinematics(getModuleTranslations());
    }

    // Update feed forward model
    if (firstCycle || driveKs.hasChanged() || driveKv.hasChanged()) {
      driveFeedForward =
          new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
    }

    // Update drive controllers
    if (driveKp.hasChanged() || driveKd.hasChanged()) {
      for (int i = 0; i < 4; i++) {
        driveControllers[i].setP(driveKp.get());
        driveControllers[i].setD(driveKd.get());
      }
    }

    // Update turn controllers
    if (firstCycle || turnKp.hasChanged() || turnKd.hasChanged()) {
      for (int i = 0; i < 4; i++) {
        turnControllers[i].setP(turnKp.get());
        turnControllers[i].setD(turnKd.get());
      }
    }

    firstCycle = false;

    // Send commands to modules
    if (DriverStation.isEnabled()) {
      if (characterizationVolts == null) {
        // Run based on setpoint
        SwerveModuleState[] setpointStates =
            kinematics.toSwerveModuleStates(setpointSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates,
            maxSpeed.get());
        for (int i = 0; i < 4; i++) {
          SwerveModuleState optimizedState =
              SwerveModuleState.optimize(setpointStates[i],
                  new Rotation2d(moduleInputs[i].turnPositionRad));
          double driveSetpointRadPerSec =
              optimizedState.speedMetersPerSecond / wheelRadius.get();

          Logger.getInstance().recordOutput(
              "Drive/Module" + Integer.toString(i) + "/DriveSetpointRadPerSec",
              driveSetpointRadPerSec);
          Logger.getInstance().recordOutput(
              "Drive/Module" + Integer.toString(i) + "/TurnSetpointRad",
              optimizedState.angle.getRadians());

          moduleIOs[i].setDriveVoltage(
              driveFeedForward.calculate(driveSetpointRadPerSec)
                  + driveControllers[i].calculate(
                      moduleInputs[i].driveVelocityRadPerSec,
                      driveSetpointRadPerSec));
          if (driveSetpointRadPerSec == 0) {
            moduleIOs[i].setTurnVoltage(0.0);
          } else {
            moduleIOs[i].setTurnVoltage(
                turnControllers[i].calculate(moduleInputs[i].turnPositionRad,
                    optimizedState.angle.getRadians()));
          }
        }

        // Log setpoint visualization
        double maxLengthPx = 150.0;
        List<Double> xPoints = new ArrayList<>();
        List<Double> yPoints = new ArrayList<>();
        List<Double> centerXs = List.of(300.0, 600.0, 300.0, 600.0);
        List<Double> centerYs = List.of(300.0, 300.0, 600.0, 600.0);
        for (int i = 0; i < 4; i++) {
          Translation2d scaledTranslation =
              new Pose2d(new Translation2d(), setpointStates[i].angle)
                  .transformBy(GeomUtil.transformFromTranslation(
                      (setpointStates[i].speedMetersPerSecond / maxSpeed.get())
                          * maxLengthPx,
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

          Logger.getInstance().recordOutput("Drive/VizX",
              xPoints.stream().mapToDouble(Double::doubleValue).toArray());
          Logger.getInstance().recordOutput("Drive/VizY",
              yPoints.stream().mapToDouble(Double::doubleValue).toArray());
        }

      } else {
        // Run based on characterization voltage
        for (int i = 0; i < 4; i++) {
          moduleIOs[i].setDriveVoltage(characterizationVolts);
          moduleIOs[i].setTurnVoltage(turnControllers[i]
              .calculate(moduleInputs[i].turnPositionRad, 0.0));
        }
      }
    } else {
      for (int i = 0; i < 4; i++) {
        moduleIOs[i].setDriveVoltage(0.0);
        moduleIOs[i].setTurnVoltage(0.0);
      }
    }

    // Update odometry
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      measuredStates[i] = new SwerveModuleState(
          moduleInputs[i].driveVelocityRadPerSec * wheelRadius.get(),
          new Rotation2d(moduleInputs[i].turnPositionRad));
    }
    ChassisSpeeds measuredSpeeds = kinematics.toChassisSpeeds(measuredStates);
    if (gyroInputs.connected) {
      pose = pose.exp(new Twist2d(
          measuredSpeeds.vxMetersPerSecond * Constants.loopPeriodSecs,
          measuredSpeeds.vyMetersPerSecond * Constants.loopPeriodSecs,
          lastYawPositionRad - gyroInputs.yawPositionRad));
    } else {
      pose = pose.exp(new Twist2d(
          measuredSpeeds.vxMetersPerSecond * Constants.loopPeriodSecs,
          measuredSpeeds.vyMetersPerSecond * Constants.loopPeriodSecs,
          measuredSpeeds.omegaRadiansPerSecond * Constants.loopPeriodSecs));
    }
    lastYawPositionRad = gyroInputs.yawPositionRad;
    Logger.getInstance().recordOutput("Odometry/Robot", new double[] {
        pose.getX(), pose.getY(), pose.getRotation().getRadians()});
  }

  public void runVelocity(ChassisSpeeds speeds) {
    setpointSpeeds = speeds;
    characterizationVolts = null;
  }

  public void runCharacterizationVolts(double volts) {
    characterizationVolts = volts;
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void resetPose(Pose2d pose) {
    this.pose = pose;
  }

  public Pose2d getPose() {
    return pose;
  }

  public Rotation2d getRotation() {
    return pose.getRotation();
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeed.get();
  }

  public double getMaxAngularSpeedRadPerSec() {
    SwerveModuleState[] states =
        kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 10000.0));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed.get());
    return kinematics.toChassisSpeeds(states).omegaRadiansPerSecond;
  }

  public Translation2d[] getModuleTranslations() {
    double halfWidthX = trackWidthX.get() / 2.0;
    double halfWidthY = trackWidthY.get() / 2.0;
    return new Translation2d[] {new Translation2d(halfWidthX, halfWidthY),
        new Translation2d(halfWidthX, -halfWidthY),
        new Translation2d(-halfWidthX, halfWidthY),
        new Translation2d(-halfWidthX, -halfWidthY)};
  }

  public double getCharacterizationVelocity() {
    return (moduleInputs[0].driveVelocityRadPerSec
        + moduleInputs[1].driveVelocityRadPerSec
        + moduleInputs[2].driveVelocityRadPerSec
        + moduleInputs[3].driveVelocityRadPerSec) / 4.0;
  }
}
