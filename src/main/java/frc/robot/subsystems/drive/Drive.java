// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;

public class Drive extends SubsystemBase {
  private final GyroIO gyroIO;
  private final GyroIOInputs gyroInputs = new GyroIOInputs();
  private final ModuleIO[] moduleIOs = new ModuleIO[4]; // FL, FR, BL, BR
  private final ModuleIOInputs[] moduleInputs =
      new ModuleIOInputs[] {new ModuleIOInputs(), new ModuleIOInputs(),
          new ModuleIOInputs(), new ModuleIOInputs()};

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
        // Initialize all of the constants here. The list is on the whiteboard.
        // You should use TunableNumbers (please ask someone to explain this).
        break;
      default:
        break;
    }
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

    // Here are all of the things the code should do here in the periodic method:

    // 1) If the values of the TunableNumbers have changed, update any necessary objects
    // (kinematics, PID controllers, FF models, etc). You may want to work on the next few steps and
    // come back to this as you go.

    // 2) On the first cycle, use the absolute encoders to reset the measured angles. You'll need to
    // record an offset value that's applied to future measurements. All of the calculations in
    // subsequent cycles should use the *relative* encoders.

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

    // 4) If running in characterization mode (and the robot is enabled), command the requested
    // voltage to the drive motors and run the PID controller to bring the angle to zero degrees.

    // 5) Update the odometry pose based on the module velocities. If the gyro is connected, use it
    // to find the angle difference. If the gyro is disconnected, estimate the angle differene based
    // on the omega componenet of the ChassisSpeeds object. The link below should give you a good
    // starting point.
    // https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/kinematics/SwerveDriveOdometry.java#L100-L106

    // 6) Enable or disable brake mode on the turn and drive motors. They should be in brake model
    // when the robot is enabled, and switch to coast when the robot is disabled *and it comes to a
    // stop*. See the example below from our 2022 code. Also discuss - why is this useful?
    // https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/subsystems/drive/Drive.java#L206-L221
  }

  /**
   * Runs the drive at the desired velocity.
   * 
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Don't actually command the modules here, since this method might be called only once at the
    // start of a command (not periodically). Instead, save the speeds and command the modules in
    // the periodic method (see the instructions above).
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /** Returns the current odometry pose. */
  public Pose2d getPose() {
    // Return the stored odometry pose. All of the calculations are in periodic, so this is just a
    // simple return.
    return new Pose2d(); // Delete this line when you start working, it just makes VSCode happy.
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    // Reset the stored odometry pose.
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    // Same idea as "runVelocity" - save the voltage and command the modules in periodic.
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    // Return the average drive velocity in radians/sec. This will be used during characterization -
    // the units are radians/sec NOT meters/sec because the feedforward model is in radians/sec.
    return 0.0; // Delete this line when you start working, it just makes VSCode happy.
  }
}
