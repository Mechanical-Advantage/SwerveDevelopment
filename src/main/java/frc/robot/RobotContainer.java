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
import frc.robot.commands.AutoDrive;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.ThreeCargoAuto;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMAX;
import frc.robot.util.Alert;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedChoosers;
import frc.robot.util.SparkMAXBurnManager;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.trajectory.Waypoint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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
    // Check if flash should be burned
    SparkMAXBurnManager.update();

    // Instantiate active subsystems
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2022S:
          drive = new Drive(new GyroIONavX(), new ModuleIOSparkMAX(0),
              new ModuleIOSparkMAX(1), new ModuleIOSparkMAX(2),
              new ModuleIOSparkMAX(3));
          break;
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
    autoRoutineMap.put("Drive Characterization",
        new AutoRoutine(AutoPosition.ORIGIN,
            new FeedForwardCharacterization(drive, true,
                new FeedForwardCharacterizationData("drive"),
                drive::runCharacterizationVolts,
                drive::getCharacterizationVelocity)));

    autoRoutineMap.put("Three Cargo",
        new AutoRoutine(AutoPosition.TARMAC_D, new ThreeCargoAuto(drive)));

    autoRoutineMap.put("Test Routine",
        new AutoRoutine(AutoPosition.ORIGIN,
            new AutoDrive(drive, List.of(
                new Waypoint(new Translation2d(), new Rotation2d(),
                    new Rotation2d()),
                new Waypoint(new Translation2d(7, 2),
                    Rotation2d.fromDegrees(90.0), Rotation2d.fromDegrees(45.0)),
                new Waypoint(new Translation2d(6, 3),
                    Rotation2d.fromDegrees(180.0), null),
                new Waypoint(new Translation2d(5, 2),
                    Rotation2d.fromDegrees(-90.0), null),
                new Waypoint(new Translation2d(9, 3),
                    Rotation2d.fromDegrees(45.0), null),
                new Waypoint(new Translation2d(9, 4), null,
                    Rotation2d.fromDegrees(180.0)),
                new Waypoint(new Translation2d(8, 5), null,
                    Rotation2d.fromDegrees(-90.0)),
                new Waypoint(new Translation2d(7, 4.5), null, null),
                new Waypoint(new Translation2d(5, 4),
                    Rotation2d.fromDegrees(45.0), Rotation2d.fromDegrees(45.0)),
                new Waypoint(new Translation2d(4, 6),
                    Rotation2d.fromDegrees(90.0), Rotation2d.fromDegrees(90.0)),
                new Waypoint(new Translation2d(3, 8),
                    Rotation2d.fromDegrees(180.0), null),
                new Waypoint(new Translation2d(2, 4),
                    Rotation2d.fromDegrees(-90.0), null)))));

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
    drive.setDefaultCommand(new DriveWithJoysticks(drive,
        () -> -controller.getLeftY(), () -> -controller.getLeftX(),
        () -> -controller.getRightX(), () -> false));
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
      drive.setPose(routine.position.getPose());
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
