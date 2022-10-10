// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 gyro;
  private final double[] xyzDps = new double[3];

  public GyroIOPigeon2() {
    switch (Constants.getRobot()) {
      case ROBOT_2022S:
        gyro = new Pigeon2(0);
        break;
      default:
        throw new RuntimeException("Invalid Robot Type");
    }
  }

  public void updateInputs(GyroIOInputs inputs) {
    // Again, the 2022 code base has good examples for reading this data. We generally prefer to use
    // "getAngle" instead of "getYaw" (what's the difference?)
    //
    // Remember to pay attention to the UNITS.
    gyro.getRawGyro(xyzDps);
    inputs.connected = gyro.getLastError().equals(ErrorCode.OK);
    inputs.positionRad = Units.degreesToRadians(gyro.getYaw());
    inputs.velocityRadPerSec = Units.degreesToRadians(xyzDps[2]);

  }
}
