package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import frc.robot.generated.TunerConstants;

public class DriveConstants {
  // TODO: guessed numbers
  public static final PIDConstants translationPIDConstants = new PIDConstants(1.5, 0, 0);
  public static final PIDConstants rotationPIDConstants = new PIDConstants(15, 0, 0);

  public static final double kSteerMaxV = 6.0; // rps
  public static final double kSteerMaxA = 4.0;
  public static final double kDriveMaxV = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude();
  public static final double kDriveMaxA = 2.0;

  public static final double kDriveFeedforward = 0.25;
  public static final double kSteerFeedforward = 2;

  public static final double rotationTolerance = 0.001;
  public static final double translationTolerance = 0.02;

  public static final double xJoystickAlignMult = 0.1;
  public static final double yJoystickAlignMult = 0.5;

  // these are IMPORTANT!
  public static final PathConstraints pathConstraints =
      new PathConstraints(4.8, 4.0, Units.rotationsToRadians(4), Units.rotationsToRadians(4));
  public static final double pathfindHandoffDistance = 6.0;

  public static final Pose2d pathfindDefaultPose =
      new Pose2d(
          FieldConstants.allianceFlip(new Translation2d(FieldConstants.fieldLength - 1.7, 2.77)),
          FieldConstants.allianceFlip(Rotation2d.kZero));
}
