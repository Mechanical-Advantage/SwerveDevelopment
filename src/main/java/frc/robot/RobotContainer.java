// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.FiveCargoAuto;
import frc.robot.commands.SixBallAuto;
import frc.robot.commands.Taxi;
import frc.robot.commands.ThreeCargoAuto;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMAX;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.DisabledInstantCommand;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedChoosers;
import frc.robot.util.SparkMAXBurnManager;

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
  private boolean isFieldRelative = true;

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
          drive = new Drive(new GyroIOPigeon2(), new ModuleIOSparkMAX(0),
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
    autoRoutineMap.put("Taxi (TA)",
        new AutoRoutine(AutoPosition.TARMAC_A, new Taxi(drive, false)));
    autoRoutineMap.put("Taxi (TB)",
        new AutoRoutine(AutoPosition.TARMAC_B, new Taxi(drive, false)));
    autoRoutineMap.put("Taxi (TC)",
        new AutoRoutine(AutoPosition.TARMAC_C, new Taxi(drive, false)));
    autoRoutineMap.put("Taxi (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D, new Taxi(drive, false)));
    autoRoutineMap.put("Taxi (FA)",
        new AutoRoutine(AutoPosition.FENDER_A, new Taxi(drive, true)));
    autoRoutineMap.put("Taxi (FB)",
        new AutoRoutine(AutoPosition.FENDER_B, new Taxi(drive, true)));
    autoRoutineMap.put("Drive Characterization",
        new AutoRoutine(AutoPosition.ORIGIN,
            new FeedForwardCharacterization(drive, true,
                new FeedForwardCharacterizationData("drive"),
                drive::runCharacterizationVolts,
                drive::getCharacterizationVelocity)));
    autoRoutineMap.put("Three Cargo",
        new AutoRoutine(AutoPosition.TARMAC_D, new ThreeCargoAuto(drive)));
    autoRoutineMap.put("Five Cargo",
        new AutoRoutine(AutoPosition.TARMAC_D, new FiveCargoAuto(drive)));
    autoRoutineMap.put("Six Cargo",
        new AutoRoutine(AutoPosition.TARMAC_D, new SixBallAuto(drive)));

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
    new Trigger(controller::getStartButton)
        .or(new Trigger(controller::getBackButton))
        .whenActive(new DisabledInstantCommand(() -> {
          isFieldRelative = !isFieldRelative;
          SmartDashboard.putBoolean("Field Relative", isFieldRelative);
        }));
    SmartDashboard.putBoolean("Field Relative", isFieldRelative);
    drive.setDefaultCommand(
        new DriveWithJoysticks(drive, () -> -controller.getLeftY(),
            () -> -controller.getLeftX(), () -> -controller.getRightX(),
            () -> !isFieldRelative, () -> choosers.getJoystickMode(),
            () -> choosers.getDemoLinearSpeedLimit(),
            () -> choosers.getDemoAngularSpeedLimit()));
    new Trigger(() -> controller.getLeftTriggerAxis() > 0.5
        || controller.getRightTriggerAxis() > 0.5)
            .whileActiveContinuous(new RunCommand(drive::goToX, drive));

    Command resetGyroCommand = new DisabledInstantCommand(() -> {
      drive.setPose(
          new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
    }, drive);
    Command rumbleCommand = new StartEndCommand(
        () -> controller.setRumble(RumbleType.kRightRumble, 0.5),
        () -> controller.setRumble(RumbleType.kRightRumble, 0.0)) {
      @Override
      public boolean runsWhenDisabled() {
        return true;
      }
    }.withTimeout(0.2);
    new Trigger(controller::getLeftBumper)
        .and(new Trigger(controller::getRightBumper))
        .whenActive(resetGyroCommand).whenActive(rumbleCommand);

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
