package frc.robot.subsystems.drive;

import frc.robot.generated.TunerConstants;

public class DriveConstants {
  // TODO: bs numbers
  public static final double kSteerMaxV = 4.0; // rps
  public static final double kSteerMaxA = 2.0;
  public static final double kDriveMaxV = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude();
  public static final double kDriveMaxA = 2.0;

  public static final double rotationTolerance = 0.001;
  public static final double translationTolerance = 0.02;

  public static final double[] kDriveFeedforwards = {}; // FL, FR, BL, BR
}
