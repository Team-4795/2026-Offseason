package frc.robot.subsystems.stateManager;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Rotation;
import static frc.robot.FieldConstants.*;

public class StateManager extends SubsystemBase {
  public enum Zone {
    GRID,
    SUBSTATION,
    NONE
  }

  private static StateManager instance;
  private State state;
  private Drive drive = Drive.getInstance();
  private Pose2d closestNodePose = new Pose2d();
  private Pose2d substationPose = new Pose2d();
  private Pose2d pathfindingNodeTargetPose;
  private Pose2d pathfindingSubstationTargetPose;
  private Translation2d scoringWallCorner = new Translation2d();
  private Translation2d scoringFeederCorner = new Translation2d();
  private Translation2d substationOuterCorner = new Translation2d();
  private Translation2d substationInnerCorner = new Translation2d();
  private Zone zone = Zone.NONE;

  private StateManager() {
    this.state = State.STOW;
    updatePoses();
    updateZones();
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

  // TODO position offset of robot when aligning
  public Translation2d getClosestNodeTranslation() {
    return closestNodePose.getTranslation();
  }

  public double getNodeDistance() {
    return closestNodePose.getTranslation().getDistance(drive.getPose().getTranslation());
  }

  public Pose2d getClosestNodePose() {
    return closestNodePose;
  }

  public Pose2d getSubstationPose() {
    return substationPose;
  }

  public Pose2d getNodePathfindingTargetPose() {
    return pathfindingNodeTargetPose;
  }

  public Pose2d getSubstationPathfindingTargetPose() {
    return pathfindingSubstationTargetPose;
  }

  public Zone getZone() {
    return this.zone;
  }

  public void updatePoses() {
    Translation2d currentPose = drive.getPose().getTranslation();
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get().equals(Alliance.Red)) {
      closestNodePose =
          new Pose2d(
              currentPose.nearest(Arrays.asList(Grids.redComplexLowTranslations)),
              Rotation2d.kZero);

      // TODO actually determine how substation logic works
      substationPose = new Pose2d(forceAllianceFlip(LoadingZone.singleSubstationTranslation), Rotation2d.k180deg);
      
      scoringWallCorner = forceAllianceFlip(Zones.wallCorner);
      scoringFeederCorner = forceAllianceFlip(Zones.feederCorner);
      substationInnerCorner = forceAllianceFlip(Zones.innerCorner);
      substationOuterCorner = forceAllianceFlip(Zones.outerCorner);

      pathfindingNodeTargetPose =
          new Pose2d(
              new Translation2d(
                  fieldLength - 1.2,
                  MathUtil.clamp(drive.getPose().getTranslation().getY(), 0.5, 4.8)),
              Rotation2d.kZero);
      pathfindingSubstationTargetPose = new Pose2d(1.45, 6.75, Rotation2d.k180deg);
    } else {
      closestNodePose =
          new Pose2d(
              currentPose.nearest(Arrays.asList(Grids.complexLowTranslations)),
              Rotation2d.k180deg);

      // TODO this one too
      substationPose = new Pose2d(LoadingZone.singleSubstationTranslation, Rotation2d.kZero);
        
      scoringWallCorner = Zones.wallCorner;
      scoringFeederCorner = Zones.feederCorner;
      substationInnerCorner = Zones.innerCorner;
      substationOuterCorner = Zones.outerCorner;

      pathfindingNodeTargetPose =
          new Pose2d(
              new Translation2d(
                  1.2, MathUtil.clamp(drive.getPose().getTranslation().getY(), 0.5, 4.8)),
              Rotation2d.k180deg);
      pathfindingSubstationTargetPose = new Pose2d(fieldLength - 1.45, 6.75, Rotation2d.kZero);
    }
  }

  // TODO clean up this bs
  public void updateZones() {
    Translation2d currentTranslation = drive.getPose().getTranslation();
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get().equals(Alliance.Red)) {
      if ((currentTranslation.getY() < scoringFeederCorner.getY()
              && currentTranslation.getY() > scoringWallCorner.getY())
          && (currentTranslation.getX() > scoringWallCorner.getX()
              && currentTranslation.getX() < scoringFeederCorner.getX())) {
        zone = Zone.GRID;
      } else if ((currentTranslation.getY() > substationOuterCorner.getY()
              && currentTranslation.getY() < substationInnerCorner.getY())
          && (currentTranslation.getX() > substationInnerCorner.getX()
              && currentTranslation.getX() < substationOuterCorner.getX())) {
        zone = Zone.SUBSTATION;
      } else {
        zone = Zone.NONE;
      }
    } else {
      if ((currentTranslation.getY() < scoringFeederCorner.getY()
              && currentTranslation.getY() > scoringWallCorner.getY())
          && (currentTranslation.getX() < scoringWallCorner.getX()
              && currentTranslation.getX() > scoringFeederCorner.getX())) {
        zone = Zone.GRID;
      } else if ((currentTranslation.getY() < substationInnerCorner.getY()
              && currentTranslation.getY() > substationOuterCorner.getY())
          && (currentTranslation.getX() > substationOuterCorner.getX()
              && currentTranslation.getX() < substationInnerCorner.getX())) {
        zone = Zone.SUBSTATION;
      } else {
        zone = Zone.NONE;
      }
    }
  }

  @Override
  public void periodic() {
    updatePoses();
    updateZones();
    Logger.recordOutput("State Manager/State", state);
    Logger.recordOutput("State Manager/Nearest Node", getClosestNodePose());
    Logger.recordOutput("State Manager/Pathfinding Node Pose", pathfindingNodeTargetPose);
    Logger.recordOutput("State Manager/Pathfinding Substation Pose", pathfindingSubstationTargetPose);
    Logger.recordOutput("State Manager/Zones/Current Zone", zone);
    Logger.recordOutput("State Manager/Zones/Scoring Wall Corner", scoringWallCorner);
    Logger.recordOutput("State Manager/Zones/Scoring Feeder Corner", scoringFeederCorner);
    Logger.recordOutput("State Manager/Zones/Substation Inner Corner", substationInnerCorner);
    Logger.recordOutput("State Manager/Zones/Substation Outer Corner", substationOuterCorner);
  }
}
