// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.TunableNumber;

public class Taxi extends CommandBase {
  private static TunableNumber longDuration =
      new TunableNumber("Taxi/LongDurationSecs");
  private static TunableNumber shortDuration =
      new TunableNumber("Taxi/ShortDurationSecs");
  private static TunableNumber speed = new TunableNumber("Taxi/Speed");

  private final Drive drive;
  private final boolean isLong;
  private final Timer timer = new Timer();

  /** Creates a new Taxi. Drives straight forwards to reliably taxi. */
  public Taxi(Drive drive, boolean isLong) {
    addRequirements(drive);
    this.drive = drive;
    this.isLong = isLong;

    longDuration.setDefault(1.25);
    shortDuration.setDefault(1.0);
    speed.setDefault(Units.inchesToMeters(80.0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.runVelocity(new ChassisSpeeds(speed.get(), 0.0, 0.0));
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(isLong ? longDuration.get() : shortDuration.get());
  }
}
