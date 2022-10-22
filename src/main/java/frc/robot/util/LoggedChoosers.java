// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.List;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Manages all SendableChoosers, including replaying values without NT. */
public class LoggedChoosers extends SubsystemBase {

  private final SendableChooser<String> joystickModeChooser =
      new SendableChooser<String>();
  private final SendableChooser<String> autoRoutineChooser =
      new SendableChooser<String>();
  private final SendableChooser<String> demoLinearSpeedLimitChooser =
      new SendableChooser<String>();
  private final SendableChooser<String> demoAngularSpeedLimitChooser =
      new SendableChooser<String>();

  private final ChooserData data = new ChooserData();

  public LoggedChoosers() {
    addOptions(joystickModeChooser, List.of("Standard", "Tank"));
    addOptions(autoRoutineChooser,
        List.of("Do Nothing", "Climb For Fun (TB)", "Climb For Fun (FA*)",
            "Score With Force (FA*)", "Score With Force (FB*)", "Taxi (TA)",
            "Taxi (TB)", "Taxi (TC)", "Taxi (TD)", "Taxi (FA)", "Taxi (FB)",
            "Drive Characterization", "Three Cargo", "Five Cargo",
            "Six Cargo"));
    addOptions(demoLinearSpeedLimitChooser, List.of("--Competition Mode--",
        "Fast Speed (70%)", "Medium Speed (30%)", "Slow Speed (15%)"));
    addOptions(demoAngularSpeedLimitChooser, List.of("--Competition Mode--",
        "Fast Speed (70%)", "Medium Speed (30%)", "Slow Speed (15%)"));

    SmartDashboard.putData("Joystick Mode", joystickModeChooser);
    SmartDashboard.putData("Auto Routine", autoRoutineChooser);
    SmartDashboard.putData("Demo/Linear Speed Limit",
        demoLinearSpeedLimitChooser);
    SmartDashboard.putData("Demo/Angular Speed Limit",
        demoAngularSpeedLimitChooser);
  }

  /** Adds a set of options to a SendableChooser. */
  private void addOptions(SendableChooser<String> chooser,
      List<String> options) {
    boolean firstOption = true;
    for (String option : options) {
      if (firstOption) {
        chooser.setDefaultOption(option, option);
        firstOption = false;
      } else {
        chooser.addOption(option, option);
      }
    }
  }

  /** Represents the selected values of all of the choosers. */
  private static class ChooserData implements LoggableInputs {
    public String joystickMode = "";
    public String autoRoutine = "";
    public String demoLinearSpeedLimit = "";
    public String demoAngularSpeedLimit = "";

    @Override
    public void toLog(LogTable table) {
      table.put("JoystickMode", joystickMode);
      table.put("AutoRoutine", autoRoutine);
      table.put("DemoLinearSpeedLimit", demoLinearSpeedLimit);
      table.put("DemoAngularSpeedLimit", demoAngularSpeedLimit);
    }

    @Override
    public void fromLog(LogTable table) {
      joystickMode = table.getString("JoystickMode", joystickMode);
      autoRoutine = table.getString("AutoRoutine", autoRoutine);
      demoLinearSpeedLimit =
          table.getString("DemoLinearSpeedLimit", demoLinearSpeedLimit);
      demoAngularSpeedLimit =
          table.getString("DemoAngularSpeedLimit", demoAngularSpeedLimit);
    }
  }

  /** Updates chooser data when not replaying, processes data w/ logging framework. */
  @Override
  public void periodic() {
    if (!Logger.getInstance().hasReplaySource()) {
      data.joystickMode = joystickModeChooser.getSelected();
      data.autoRoutine = autoRoutineChooser.getSelected();
      data.demoLinearSpeedLimit = demoLinearSpeedLimitChooser.getSelected();
      data.demoAngularSpeedLimit = demoAngularSpeedLimitChooser.getSelected();
    }
    Logger.getInstance().processInputs("Choosers", data);
  }

  public String getJoystickMode() {
    return data.joystickMode;
  }

  public String getAutoRoutine() {
    return data.autoRoutine;
  }

  public double getDemoLinearSpeedLimit() {
    switch (data.demoLinearSpeedLimit) {
      default:
        return 1;
      case "Fast Speed (70%)":
        return 0.7;
      case "Medium Speed (30%)":
        return 0.3;
      case "Slow Speed (15%)":
        return 0.15;
    }
  }

  public double getDemoAngularSpeedLimit() {
    switch (data.demoAngularSpeedLimit) {
      default:
        return 1;
      case "Fast Speed (70%)":
        return 0.7;
      case "Medium Speed (30%)":
        return 0.3;
      case "Slow Speed (15%)":
        return 0.15;
    }
  }
}
