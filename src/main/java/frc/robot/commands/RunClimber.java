// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class RunClimber extends CommandBase {
  private static final double deadband = 0.1;

  private final Climber climber;
  private final Supplier<Double> axisSupplier;

  /** Creates a new RunClimber. */
  public RunClimber(Climber climber, Supplier<Double> axisSupplier) {
    addRequirements(climber);
    this.climber = climber;
    this.axisSupplier = axisSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double value = axisSupplier.get();
    value = MathUtil.applyDeadband(value, deadband);
    value = Math.copySign(value * value, value);
    climber.runPercent(value);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
