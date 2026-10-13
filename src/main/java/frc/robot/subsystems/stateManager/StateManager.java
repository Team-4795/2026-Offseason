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
  private Pose2d closestNode = new Pose2d();
  public Pose2d pathfindTargetPose = new Pose2d();
  private Translation2d wallCorner = new Translation2d();
  private Translation2d feederCorner = new Translation2d();

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
  // public Pose2d getTestingAlignPose() {
  //   return new Pose2d(
  //       new Translation2d(
  //           FieldConstants.Community.chargingStationOuterX,
  //           FieldConstants.Community.chargingStationRightY),
  //       Rotation2d.kZero);
  // }

  // TODO position offset of robot when aligning
  public Translation2d getClosestNodeTranslation() {
    return closestNode.getTranslation();
  }

  public double getNodeDistance() {
    return closestNode.getTranslation().getDistance(drive.getPose().getTranslation());
  }

  public Pose2d getClosestNodePose() {
    return closestNode;
  }

  public Pose2d getPathfindingTargetPose() {
    return pathfindTargetPose;
  }

  public void updateClosestNodeTranslation() {
    Translation2d currentPose = drive.getPose().getTranslation();
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get().equals(Alliance.Red)) {
      closestNode =
          new Pose2d(
              currentPose.nearest(Arrays.asList(FieldConstants.Grids.redComplexLowTranslations)),
              Rotation2d.kZero);
      wallCorner = FieldConstants.forceAllianceFlip(FieldConstants.Grids.wallCorner);
      feederCorner = FieldConstants.forceAllianceFlip(FieldConstants.Grids.feederCorner);
      pathfindTargetPose =
          new Pose2d(
              new Translation2d(2.2, drive.getPose().getTranslation().getY()), Rotation2d.kZero);
    } else {
      closestNode =
          new Pose2d(
              currentPose.nearest(Arrays.asList(FieldConstants.Grids.complexLowTranslations)),
              Rotation2d.k180deg);
      wallCorner = FieldConstants.Grids.wallCorner;
      feederCorner = FieldConstants.Grids.feederCorner;
      pathfindTargetPose =
          new Pose2d(
              new Translation2d(
                  FieldConstants.fieldLength - 2.2, drive.getPose().getTranslation().getY()),
              Rotation2d.k180deg);
    }
  }

  // /** don't use unless you know what you're doing...
  //  * auto align relies on this thing not being called periodically
  //  */
  // public void updatePathfindingDefaultPose() {
  //   pathfindDefaultPose =
  //           new Pose2d(FieldConstants.allianceFlip(new Translation2d(FieldConstants.fieldLength -
  // 1.7, 2.77)), FieldConstants.allianceFlip(Rotation2d.kZero));
  // }

  public boolean withinScoringZone(Translation2d currentTranslation) {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get().equals(Alliance.Red)) {
      return ((currentTranslation.getY() < feederCorner.getY()
              && currentTranslation.getY() > wallCorner.getY())
          && (currentTranslation.getX() > wallCorner.getX()
              && currentTranslation.getX() < feederCorner.getX()));
    } else {
      return ((currentTranslation.getY() < feederCorner.getY()
              && currentTranslation.getY() > wallCorner.getY())
          && (currentTranslation.getX() < wallCorner.getX()
              && currentTranslation.getX() > feederCorner.getX()));
    }
  }

  @Override
  public void periodic() {
    updateClosestNodeTranslation();
    Logger.recordOutput("StateManager/State", state);
    Logger.recordOutput("StateManager/Nearest Node", getClosestNodePose());
    // Logger.recordOutput("StateManager/Pathfinding Default Pose", pathfindDefaultPose);
    Logger.recordOutput(
        "Zones/In scoring zone?", withinScoringZone(drive.getPose().getTranslation()));
    Logger.recordOutput("Zones/Scoring Wall Corner", wallCorner);
    Logger.recordOutput("Zones/Scoring Feeder Corner", feederCorner);
  }
}
