// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

public class GyroIONavX implements GyroIO {
  private AHRS gyro;

  public GyroIONavX() {

    // The navX is the standard gyro that we've used on all of our existing robots. It's possible
    // that the swerve base will use a different sensor, but please fill in this implementation as a
    // backup (and because it's good practice).

    switch (Constants.getRobot()) {
      case ROBOT_2022S:
        // Instantiate the navX - use the 2022 code as an example.
        gyro = new AHRS(SPI.Port.kMXP);
        break;
      default:
        throw new RuntimeException("Invalid robot for GyroIONavX");
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Again, the 2022 code base has good examples for reading this data. We generally prefer to use
    // "getAngle" instead of "getYaw" (what's the difference?)
    //
    // Remember to pay attention to the UNITS.
    inputs.connected = gyro.isConnected();
    inputs.positionRad = Units.degreesToRadians(gyro.getAngle());
    inputs.velocityRadPerSec = Units.degreesToRadians(gyro.getRate());
  }
}
