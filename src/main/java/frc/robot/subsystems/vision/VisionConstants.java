package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

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
