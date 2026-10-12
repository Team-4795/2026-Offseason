package frc.robot.subsystems.drive;

import frc.robot.generated.TunerConstants;

public class DriveConstants {
  // TODO: guessed numbers
  public static final double kSteerMaxV = 6.0; // rps
  public static final double kSteerMaxA = 4.0;
  public static final double kDriveMaxV = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude();
  public static final double kDriveMaxA = 2.0;

  public static final double kDriveFeedforward = 0.25;
  public static final double kSteerFeedforward = 2;

  public static final double rotationTolerance = 0.001;
  public static final double translationTolerance = 0.02;
}
