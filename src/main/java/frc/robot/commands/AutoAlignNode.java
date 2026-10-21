package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.stateManager.StateManager;
import org.littletonrobotics.junction.Logger;

public class AutoAlignNode extends Command {
  private Drive drive;
  private StateManager manager = StateManager.getInstance();
  private Pose2d currentPose;
  private Pose2d targetPose;
  // private PPHolonomicDriveController controller = new
  // PPHolonomicDriveController(DriveConstants.translationPIDConstants,
  // DriveConstants.rotationPIDConstants);
  private PIDController translationController = new PIDController(5, 0, 0);
  private ProfiledPIDController rotationController =
      new ProfiledPIDController(
          15,
          0,
          1.5,
          new TrapezoidProfile.Constraints(
              Units.rotationsToRadians(DriveConstants.kSteerMaxV),
              Units.rotationsToRadians(DriveConstants.kSteerMaxA)));

  public AutoAlignNode(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.resetSetpoints();
    currentPose = drive.getPose();
    targetPose = manager.getClosestNodePoseWithUpdate();
    rotationController.reset(
        drive.getRotation().getRadians(), drive.getChassisSpeeds().omegaRadiansPerSecond);
    translationController.reset();
  }

  @Override
  public void execute() {
    // setup
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(DriveConstants.rotationTolerance);
    translationController.setTolerance(DriveConstants.translationTolerance);

    currentPose = drive.getPose();
    targetPose = manager.getClosestNodePoseWithUpdate();

    // stop moving if within tolerance
    // idk if these lines are even necessary
    double tMult = translationController.atSetpoint() ? 0.0 : 1.0;
    double rMult = rotationController.atSetpoint() ? 0.0 : 1.0;

    // calculating outputs

    double tError = targetPose.getTranslation().getDistance(currentPose.getTranslation());
    double rError = targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians();

    double rotationOutput =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    double translationOutput = translationController.calculate(tError);

    Rotation2d direction =
        new Rotation2d(
            currentPose.getX() - targetPose.getX(), currentPose.getY() - targetPose.getY());

    double translationX = direction.getCos() * translationOutput;
    double translationY = direction.getSin() * translationOutput;

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(tMult * translationX, tMult * translationY, rMult * rotationOutput),
            currentPose.getRotation());

    drive.runVelocity(speeds);
    // drive.runVelocityWithSetpointGen(speeds);

    Logger.recordOutput("Auto Align/Target Pose", targetPose);
    Logger.recordOutput("Auto Align/vX", translationX);
    Logger.recordOutput("Auto Align/vY", translationY);
    Logger.recordOutput("Auto Align/omega", rotationOutput);
    Logger.recordOutput("Auto Align/Translation Error (m)", tError);
    Logger.recordOutput("Auto Align/Rotation Error (rad)", rError);
    Logger.recordOutput("Auto Align/tMult", tMult);
    Logger.recordOutput("Auto Align/rMult", rMult);
  }
}
