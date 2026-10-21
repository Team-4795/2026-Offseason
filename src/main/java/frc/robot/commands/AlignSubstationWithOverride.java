package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.stateManager.StateManager;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AlignSubstationWithOverride extends Command {
  private Drive drive;
  private StateManager manager = StateManager.getInstance();
  private Pose2d currentPose;
  private Pose2d targetPose;
  private PIDController translationController = new PIDController(5, 0, 0);
  private ProfiledPIDController rotationController =
      new ProfiledPIDController(
          15,
          0,
          1.5,
          new TrapezoidProfile.Constraints(
              Units.rotationsToRadians(DriveConstants.kSteerMaxV),
              Units.rotationsToRadians(DriveConstants.kSteerMaxA)));

  private DoubleSupplier xAxis;
  private DoubleSupplier yAxis;

  private double overrideMult;

  public AlignSubstationWithOverride(Drive drive, DoubleSupplier x, DoubleSupplier y) {
    this.drive = drive;
    xAxis = x;
    yAxis = y;
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
    targetPose = manager.getSubstationPoseWithUpdate();

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

    // multiplier applied to PID controller outputs and driver inputs
    // depending on how much driver joystick is moved
    double x = Math.abs(xAxis.getAsDouble());
    double y = Math.abs(yAxis.getAsDouble());

    // from PID/ff
    ChassisSpeeds feedbackSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                tMult * translationX * (1.0 - x),
                tMult * translationY * (1.0 - y),
                rMult * rotationOutput),
            currentPose.getRotation());

    // from driver controller
    Translation2d linearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(xAxis.getAsDouble(), yAxis.getAsDouble());

    ChassisSpeeds driverSpeeds =
        new ChassisSpeeds(
            linearVelocity.getX()
                * drive.getMaxLinearSpeedMetersPerSec()
                * x
                * DriveConstants.xJoystickAlignMult,
            linearVelocity.getY()
                * drive.getMaxLinearSpeedMetersPerSec()
                * y
                * DriveConstants.yJoystickAlignMult,
            0);

    // add them up and run them!
    ChassisSpeeds totalSpeeds = driverSpeeds.plus(feedbackSpeeds);
    drive.runVelocity(totalSpeeds);

    // drive.runVelocityWithSetpointGen(speeds);

    Logger.recordOutput("Align w. Override/Target Pose", targetPose);
    Logger.recordOutput("Align w. Override/vX", translationX);
    Logger.recordOutput("Align w. Override/vY", translationY);
    Logger.recordOutput("Align w. Override/omega", rotationOutput);
    Logger.recordOutput("Align w. Override/Translation Error (m)", tError);
    Logger.recordOutput("Align w. Override/Rotation Error (rad)", rError);
    Logger.recordOutput("Align w. Override/tMult", tMult);
    Logger.recordOutput("Align w. Override/rMult", rMult);
    Logger.recordOutput("Align w. Override/OverrideMult", overrideMult);
  }
}
