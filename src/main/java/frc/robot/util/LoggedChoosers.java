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

  private final SendableChooser<String> autoRoutineChooser =
      new SendableChooser<String>();
  private final SendableChooser<String> demoSpeedLimitChooser =
      new SendableChooser<String>();
  private final SendableChooser<String> demoDriveModeChooser =
      new SendableChooser<String>();

  private final ChooserData data = new ChooserData();

  public LoggedChoosers() {
    addOptions(autoRoutineChooser, List.of("Do Nothing", "Test Routine",
        "Drive Characterization", "Three Cargo", "Five Cargo", "Six Cargo"));
    addOptions(demoSpeedLimitChooser, List.of("--Competition Mode--",
        "Fast Speed (70%)", "Medium Speed (30%)", "Slow Speed (15%)"));
    addOptions(demoDriveModeChooser, List.of("--Competition Mode--", "Tank"));

    SmartDashboard.putData("Auto Routine", autoRoutineChooser);
    SmartDashboard.putData("Demo/Speed Limit", demoSpeedLimitChooser);
    SmartDashboard.putData("Demo/Drive Mode", demoDriveModeChooser);
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
    public String autoRoutine = "";
    public String demoSpeedLimit = "";
    public String demoDriveMode = "";

    @Override
    public void toLog(LogTable table) {
      table.put("AutoRoutine", autoRoutine);
      table.put("DemoSpeedLimit", demoSpeedLimit);
      table.put("DemoDriveMode", demoSpeedLimit);
    }

    @Override
    public void fromLog(LogTable table) {
      autoRoutine = table.getString("AutoRoutine", autoRoutine);
      demoSpeedLimit = table.getString("DemoSpeedLimit", demoSpeedLimit);
      demoDriveMode = table.getString("DemoDriveMode", demoDriveMode);
    }
  }

  /** Updates chooser data when not replaying, processes data w/ logging framework. */
  @Override
  public void periodic() {
    if (!Logger.getInstance().hasReplaySource()) {
      data.autoRoutine = autoRoutineChooser.getSelected();
      data.demoSpeedLimit = demoSpeedLimitChooser.getSelected();
      data.demoDriveMode = demoDriveModeChooser.getSelected();
    }
    Logger.getInstance().processInputs("Choosers", data);
  }

  public String getAutoRoutine() {
    return data.autoRoutine;
  }

  public double getDemoSpeedLimit() {
    switch (data.demoSpeedLimit) {
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

  public boolean getDemoTankMode() {
    return data.demoDriveMode != "Tank";
  }
}
