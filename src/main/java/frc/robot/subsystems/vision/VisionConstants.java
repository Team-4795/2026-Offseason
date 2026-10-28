package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // Placeholders
  public static final double BORDER_MARGIN = 0.5;
  public static final double Z_MARGIN = 0.35;

  public static final double XY_SINGLE_STDEV = 0.08;
  public static final double XY_MULTIPLE_STDEV = 0.04;

  // we will probably run right and left off of one opi
  // and front on its own opi
  public static final String[] CAM_NAMES = {"Right Cam", "Front Cam", "Left Cam"};

  public static final Transform3d[] CAM_POSES =
      new Transform3d[] {
        // Right Cam
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(
                Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))),

        // Front Cam
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(
                Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))),

        // Left Cam
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(
                Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))
      };

  public static AprilTagFieldLayout FIELD_LAYOUT;

  static {
    FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2023ChargedUp);
    FIELD_LAYOUT.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
  }
}
