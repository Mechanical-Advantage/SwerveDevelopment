// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;

public class Drive extends SubsystemBase {
  private final GyroIO gyroIO;
  private final ModuleIO flModuleIO;
  private final ModuleIO frModuleIO;
  private final ModuleIO blModuleIO;
  private final ModuleIO brModuleIO;
  private final GyroIOInputs gyroInputs = new GyroIOInputs();

  /** Creates a new Drive. */
  public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO,
      ModuleIO blModuleIO, ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    this.flModuleIO = flModuleIO;
    this.frModuleIO = frModuleIO;
    this.blModuleIO = blModuleIO;
    this.brModuleIO = brModuleIO;


  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    gyroIO.updateInputs(gyroInputs);
    Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
  }
}
