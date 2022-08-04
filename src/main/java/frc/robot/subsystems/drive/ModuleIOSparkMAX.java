// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.util.SparkMAXBurnManager;

public class ModuleIOSparkMAX implements ModuleIO {
  private final CANSparkMax flSparkMax;
  private final CANSparkMax frSparkMax;
  private final CANSparkMax blSparkMax;
  private final CANSparkMax brSparkMax;

  private Encoder flExternalEncoder;
  private Encoder frExternalEncoder;
  private Encoder blExternalEncoder;
  private Encoder brExternalEncoder;

  private RelativeEncoder flInternalEncoder;
  private RelativeEncoder frInternalEncoder;
  private RelativeEncoder blInternalEncoder;
  private RelativeEncoder brInternalEncoder;

  private boolean isEncoderInverted = false;
  private boolean hasExternalEncoder = false;

  private boolean flMotorInverted = false;
  private boolean frMotorInverted = false;
  private boolean blMotorInverted = false;
  private boolean brMotorInverted = false;

  public ModuleIOSparkMAX(int index) {

    switch (Constants.getRobot()) {
      case ROBOT_SIMBOT:
        flSparkMax = new CANSparkMax(1, MotorType.kBrushless);
        frSparkMax = new CANSparkMax(2, MotorType.kBrushless);
        blSparkMax = new CANSparkMax(3, MotorType.kBrushless);
        brSparkMax = new CANSparkMax(4, MotorType.kBrushless);
        // Here, you should set any constants that are the same for every module (i.e. the gear
        // ratios AKA "after encoder reductions").
        final double GEAR_RATIO = 0.0;
        switch (index) { // Each module has its own configuration
          case 0:
            // Instantiate the SparkMAX objects with CAN IDs, plus:
            // 1) Whether each motor is inverted
            // 2) Whether the absolute encoder is inverted
            // 3) The offset rotation for the absolute encoder (see the explanation in
            // updateInputs).

            flExternalEncoder = new Encoder(0, 0); // will have to change digital input channels for
                                                   // these
            frExternalEncoder = new Encoder(0, 0);
            blExternalEncoder = new Encoder(0, 0);
            brExternalEncoder = new Encoder(0, 0);

            break;
          case 1:
            // Same as above, this module will have a different config
            flExternalEncoder = new Encoder(0, 0); // will have to change digital input channels for
                                                   // these
            frExternalEncoder = new Encoder(0, 0);
            blExternalEncoder = new Encoder(0, 0);
            brExternalEncoder = new Encoder(0, 0);

            break;
          case 2:
            // Same as above, this module will have a different config
            flExternalEncoder = new Encoder(0, 0); // will have to change digital input channels for
                                                   // these
            frExternalEncoder = new Encoder(0, 0);
            blExternalEncoder = new Encoder(0, 0);
            brExternalEncoder = new Encoder(0, 0);

            break;
          case 3:
            // Same as above, this module will have a different config
            flExternalEncoder = new Encoder(0, 0);
            frExternalEncoder = new Encoder(0, 0);
            blExternalEncoder = new Encoder(0, 0); // will have to change digital input channels for
                                                   // these
            brExternalEncoder = new Encoder(0, 0);

            break;
        }
        break;
      default:
        flSparkMax = new CANSparkMax(1, MotorType.kBrushless);
        frSparkMax = new CANSparkMax(2, MotorType.kBrushless);
        blSparkMax = new CANSparkMax(3, MotorType.kBrushless);
        brSparkMax = new CANSparkMax(4, MotorType.kBrushless);
        throw new RuntimeException("Invalid robot for ModuleIOSparkMAX");
    }

    if (SparkMAXBurnManager.shouldBurn()) {
      // Here, you should restore the SparkMAXs to factory defaults. If anyone remember what the
      // SparkMAXBurnManager is for, please talk about it.
    }

    // Configure the SparkMAXs here and retrieve the relative encoder objects. See the example below
    // to give you an idea.
    // https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/subsystems/climber/ClimberIOSparkMAX.java

    // The absolute encoder is a little different than anything in the 2022 code. Uncomment the code
    // below that retrieves the absolute encoder object:
    //
    // absoluteEncoder = turnMotor.getAnalog(Mode.kAbsolute);
    // absoluteEncoder.setInverted(???);

    if (SparkMAXBurnManager.shouldBurn()) {
      // Here, you should burn the flash on the SparkMAXs.
    }
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Check the 2022 code for examples of retrieving most of these values. Remember to check the
    // UNITS on everything - the relative encoder objects don't return radians and radians/sec.

    // Again, the absolute encoder is a little different than anything in the 2022 code. Our plan is
    // to use the Thrifty Absolute Magnetic Encoder (Google it for details and the manual). It
    // outputs a voltage that needs to be converted to radians (0V-5V). The offset rotation is
    // because we don't know the orientation of the magnet - there's a constant offset beteween
    // where "the module is pointed forwards" and "the encoder reads zero radians". While I haven't
    // tested it yet, I believe the line below has all of the correct conversions...
    //
    // inputs.turnAbsolutePositionRad = new Rotation2d((absoluteEncoder.getPosition() / 5.0) * 2.0 *
    // Math.PI).minus(absoluteEncoderOffset).getRadians();
  }

  // NOTE: For all of the methods below, check the 2022 code for examples.

  public void setDriveVoltage(double volts) {}

  public void setTurnVoltage(double volts) {}

  public void setDriveBrakeMode(boolean enable) {}

  public void setTurnBrakeMode(boolean enable) {}
}
