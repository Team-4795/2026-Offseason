package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class AutoAlignTest extends Command {
  private Drive drive = Drive.getInstance();
  private ProfiledPIDController translationControllerX =
      new ProfiledPIDController(
          5,
          0,
          0,
          new TrapezoidProfile.Constraints(DriveConstants.kDriveMaxV, DriveConstants.kDriveMaxA));
  private ProfiledPIDController translationControllerY =
      new ProfiledPIDController(
          5,
          0,
          0,
          new TrapezoidProfile.Constraints(DriveConstants.kDriveMaxV, DriveConstants.kDriveMaxA));
  private ProfiledPIDController rotationController =
      new ProfiledPIDController(
          15,
          0,
          1.5,
          new TrapezoidProfile.Constraints(DriveConstants.kSteerMaxV, DriveConstants.kSteerMaxA));

  public AutoAlignTest() {
    addRequirements(drive);
  }

  @Override
  public void execute() {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(DriveConstants.rotationTolerance);
    translationControllerX.setTolerance(DriveConstants.translationTolerance);
    translationControllerY.setTolerance(DriveConstants.translationTolerance);

    Pose2d currentPose = drive.getPose();
    Pose2d targetPose =
        new Pose2d(
            new Translation2d(
                FieldConstants.Community.chargingStationOuterX,
                FieldConstants.Community.chargingStationRightY),
            Rotation2d.kZero);

    double rotationOutput = rotationController.calculate(currentPose.getRotation().getRadians());
    double translationOutputX =
        translationControllerX.calculate(currentPose.getX(), targetPose.getX());
    double translationOutputY =
        translationControllerY.calculate(currentPose.getY(), targetPose.getY());

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(translationOutputX, translationOutputY, rotationOutput), currentPose.getRotation());
    drive.driveWithSetpoint(speeds);
    Logger.recordOutput("Auto Align/Target Pose", targetPose);
    Logger.recordOutput("Auto Align/vX", translationOutputX);
    Logger.recordOutput("Auto Align/vY", translationOutputY);
    Logger.recordOutput("Auto Align/omega", rotationOutput);
  }
}
