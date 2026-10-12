package frc.robot.subsystems.stateManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class StateManager extends SubsystemBase {
  private static StateManager instance;
  private State state;
  private Drive drive = Drive.getInstance();
  private Translation2d closestNode = new Translation2d();

  private StateManager() {
    // TODO temporary
    this.state = State.STOW;
  }

  public static StateManager getInstance() {
    return instance;
  }

  public static StateManager initialize() {
    if (instance == null) {
      instance = new StateManager();
    }
    return instance;
  }

  public void setState(State s) {
    state = s;
  }

  public State getState() {
    return state;
  }

  /** testing purposes only */
  public Pose2d getTestingAlignPose() {
    return new Pose2d(
        new Translation2d(
            FieldConstants.Community.chargingStationOuterX,
            FieldConstants.Community.chargingStationRightY),
        Rotation2d.kZero);
  }

  // TODO position offset of robot when aligning
  public Translation2d getClosestNodeTranslation() {
    return closestNode;
  }

  public double getNodeDistance() {
    return closestNode.getDistance(drive.getPose().getTranslation());
  }

  public Pose2d getClosestNodePose() {
    return new Pose2d(closestNode, FieldConstants.allianceFlip(Rotation2d.kZero));
  }

  public void updateClosestNodeTranslation() {
    Translation2d currentPose = drive.getPose().getTranslation();
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get().equals(Alliance.Red))
        closestNode =
            currentPose.nearest(Arrays.asList(FieldConstants.Grids.redComplexLowTranslations));
      else
        closestNode =
            currentPose.nearest(Arrays.asList(FieldConstants.Grids.complexLowTranslations));
    }
  }

  @Override
  public void periodic() {
    updateClosestNodeTranslation();
    Logger.recordOutput("StateManager/State", state);
    Logger.recordOutput("StateManager/Nearest Node", getClosestNodePose());
  }
}
