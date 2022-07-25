// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.util.Alert;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedChoosers;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.trajectory.Waypoint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private Drive drive;

  // OI objects
  private XboxController controller = new XboxController(0);

  // Choosers
  private final LoggedChoosers choosers = new LoggedChoosers();
  private final Map<String, AutoRoutine> autoRoutineMap =
      new HashMap<String, AutoRoutine>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Instantiate active subsystems
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_SIMBOT:
          drive = new Drive(new GyroIO() {}, new ModuleIOSim(),
              new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
          break;
        default:
          break;
      }
    }

    // Instantiate missing subsystems
    drive = drive != null ? drive
        : new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {},
            new ModuleIO() {}, new ModuleIO() {});

    // Set up auto routines
    autoRoutineMap.put("Do Nothing",
        new AutoRoutine(AutoPosition.ORIGIN, new InstantCommand()));

    autoRoutineMap.put("Test Routine",
        new AutoRoutine(AutoPosition.ORIGIN,
            new FollowTrajectory(drive,
                List.of(Waypoint.fromHolonomicPose(new Pose2d()),
                    Waypoint.fromHolonomicPose(
                        new Pose2d(5.5, 2.0, Rotation2d.fromDegrees(180.0))),
                    Waypoint.fromHolonomicPose(
                        new Pose2d(3.0, 6.7, Rotation2d.fromDegrees(0.0)),
                        Rotation2d.fromDegrees(180.0))))));

    autoRoutineMap.put("Five Cargo Auto",
        new AutoRoutine(AutoPosition.TARMAC_D, new SequentialCommandGroup(
            new FollowTrajectory(drive,
                List.of(
                    Waypoint.fromHolonomicPose(AutoPosition.TARMAC_D.getPose()),
                    Waypoint.fromHolonomicPose(AutoConstants.cargoPositions
                        .get(AutoPosition.TARMAC_D)))),
            new FollowTrajectory(drive, List.of(
                Waypoint.fromHolonomicPose(
                    AutoConstants.cargoPositions.get(AutoPosition.TARMAC_D)),
                Waypoint.fromHolonomicPose(
                    AutoConstants.cargoPositions.get(AutoPosition.TARMAC_C),
                    AutoConstants.cargoPositions.get(AutoPosition.TARMAC_C)
                        .getRotation()))),
            new FollowTrajectory(drive, List.of(
                Waypoint.fromHolonomicPose(
                    AutoConstants.cargoPositions.get(AutoPosition.TARMAC_C)),
                Waypoint.fromHolonomicPose(AutoConstants.terminalCargoPosition,
                    AutoConstants.terminalCargoPosition.getRotation()))),
            new FollowTrajectory(drive, List.of(
                Waypoint.fromHolonomicPose(AutoConstants.terminalCargoPosition),
                Waypoint
                    .fromHolonomicPose(AutoConstants.calcAimedPose(new Pose2d(
                        new Translation2d(5.0, 2.0), new Rotation2d()))))))));

    FeedForwardCharacterizationData characterizationData =
        new FeedForwardCharacterizationData("Drive");
    autoRoutineMap.put("Drive Characterization",
        new AutoRoutine(AutoPosition.ORIGIN,
            new FeedForwardCharacterization(drive, true, characterizationData,
                drive::runCharacterizationVolts,
                drive::getCharacterizationVelocity)));

    // Alert if in tuning mode
    if (Constants.tuningMode) {
      new Alert("Tuning mode active, expect decreased network performance.",
          AlertType.INFO).set(true);
    }

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    DriveWithJoysticks driveWithJoysticks =
        new DriveWithJoysticks(drive, () -> -controller.getLeftY(),
            () -> -controller.getLeftX(), () -> -controller.getRawAxis(2));
    drive.setDefaultCommand(driveWithJoysticks);
    new Trigger(controller::getLeftBumper)
        .whenActive(driveWithJoysticks::toggleFieldRelative);
    new Trigger(controller::getRightBumper)
        .whenActive(() -> driveWithJoysticks.setAutoAim(true))
        .whenInactive(() -> driveWithJoysticks.setAutoAim(false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String routineString = choosers.getAutoRoutine();
    if (autoRoutineMap.containsKey(routineString)) {
      AutoRoutine routine = autoRoutineMap.get(routineString);
      drive.resetPose(routine.position.getPose());
      return routine.command;

    } else {
      DriverStation.reportError("Unknown auto routine: '" + routineString + "'",
          false);
      return null;
    }
  }

  private static class AutoRoutine {
    public final AutoPosition position;
    public final Command command;

    public AutoRoutine(AutoPosition position, Command command) {
      this.position = position;
      this.command = command;
    }
  }

  public static enum AutoPosition {
    ORIGIN, TARMAC_A, TARMAC_B, TARMAC_C, TARMAC_D, FENDER_A, FENDER_B;

    public Pose2d getPose() {
      switch (this) {
        case ORIGIN:
          return new Pose2d();
        case TARMAC_A:
          return FieldConstants.referenceA
              .transformBy(GeomUtil.transformFromTranslation(-0.5, 0.7));
        case TARMAC_B:
          return FieldConstants.referenceB
              .transformBy(GeomUtil.transformFromTranslation(-0.5, -0.2));
        case TARMAC_C:
          return FieldConstants.referenceC
              .transformBy(GeomUtil.transformFromTranslation(-0.5, -0.1));
        case TARMAC_D:
          return FieldConstants.referenceD
              .transformBy(GeomUtil.transformFromTranslation(-0.5, -0.7));
        case FENDER_A:
          return FieldConstants.fenderA
              .transformBy(GeomUtil.transformFromTranslation(0.5, 0.0));
        case FENDER_B:
          return FieldConstants.fenderB
              .transformBy(GeomUtil.transformFromTranslation(0.5, 0.0));
        default:
          return new Pose2d();
      }
    }
  }
}
