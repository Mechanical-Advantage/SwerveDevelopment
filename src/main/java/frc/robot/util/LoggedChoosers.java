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

  private final ChooserData data = new ChooserData();

  public LoggedChoosers() {
    addOptions(autoRoutineChooser, List.of("Do Nothing", "Test Routine",
        "Drive Characterization", "Three Cargo", "Five Cargo", "Six Cargo"));

    SmartDashboard.putData("Auto Routine", autoRoutineChooser);
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

    @Override
    public void toLog(LogTable table) {
      table.put("AutoRoutine", autoRoutine);
    }

    @Override
    public void fromLog(LogTable table) {
      autoRoutine = table.getString("AutoRoutine", autoRoutine);
    }
  }

  /** Updates chooser data when not replaying, processes data w/ logging framework. */
  @Override
  public void periodic() {
    if (!Logger.getInstance().hasReplaySource()) {
      data.autoRoutine = autoRoutineChooser.getSelected();
    }
    Logger.getInstance().processInputs("Choosers", data);
  }

  public String getAutoRoutine() {
    return data.autoRoutine;
  }
}
