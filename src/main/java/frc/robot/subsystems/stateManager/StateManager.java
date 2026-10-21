package frc.robot.subsystems.stateManager;

import static frc.robot.FieldConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants.FieldZones;
import frc.robot.FieldConstants.Grids;
import frc.robot.FieldConstants.LoadingZone;
import frc.robot.subsystems.drive.Drive;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class StateManager extends SubsystemBase {
  public enum Zone {
    GRID,
    SUBSTATION,
    NONE
  }

  public enum ScoringMode {
    CUBE,
    CONE,
    NONE
  }

  private static StateManager instance;

  // poses
  private Pose2d currentPose;
  private Pose2d closestNodePose = new Pose2d();
  private Pose2d substationPose = new Pose2d();
  private Pose2d pathfindingNodeTargetPose;
  private Pose2d pathfindingSubstationTargetPose;

  // zones
  private Translation2d scoringWallCorner = new Translation2d();
  private Translation2d scoringFeederCorner = new Translation2d();
  private Translation2d substationOuterCorner = new Translation2d();
  private Translation2d substationInnerCorner = new Translation2d();
  private Zone zone = Zone.NONE;

  // scoring states
  private ScoringMode scoringMode = ScoringMode.NONE;

  private Setpoint[] coneSetpoints = {Setpoint.CONE_LOW, Setpoint.CONE_MID, Setpoint.CONE_HIGH};
  private Setpoint[] cubeSetpoints = {Setpoint.CUBE_LOW, Setpoint.CUBE_MID, Setpoint.CUBE_HIGH};

  private Setpoint currentSetpoint = Setpoint.STOW;
  private int scoringLevel = 0; // 0 - 2 inclusive, corresponding to setpoint array elements

  private Translation2d testingSubstationAlignPos = new Translation2d(fieldLength - 1.45, 6.75);

  private StateManager() {
    this.currentSetpoint = Setpoint.STOW;
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

  public void setScoringLevel(int level) {
    scoringLevel = level;
  }

  public void incrementScoringLevel() {
    if (scoringLevel < coneSetpoints.length - 1) scoringLevel++;
  }

  public void decrementScoringLevel() {
    if (scoringLevel > 0) scoringLevel--;
  }

  public void setCurrentSetpoint(Setpoint s) {
    if (currentSetpoint != s) currentSetpoint = s;
  }

  public void setSetpointTesting() {
    setCurrentSetpoint(Setpoint.CONE_MID);
  }

  public void setScoringMode(ScoringMode m) {
    if (scoringMode != m) scoringMode = m;
  }

  // TODO position offset of robot when aligning
  public Translation2d getClosestNodeTranslation() {
    return closestNodePose.getTranslation();
  }

  public double getNodeDistance() {
    return closestNodePose.getTranslation().getDistance(currentPose.getTranslation());
  }

  public Pose2d getClosestNodePose() {
    return closestNodePose;
  }

  public Pose2d getSubstationPose() {
    return substationPose;
  }

  public Pose2d getNodePathfindingTargetPose() {
    currentPose = Drive.getInstance().getPose();
    pathfindingNodeTargetPose =
        allianceFlip(
            new Pose2d(
                (new Translation2d(
                    1.2, MathUtil.clamp(currentPose.getTranslation().getY(), 0.5, 4.8))),
                Rotation2d.k180deg));
    return pathfindingNodeTargetPose;
  }

  public Pose2d getSubstationPathfindingTargetPose() {
    pathfindingSubstationTargetPose =
        allianceFlip(new Pose2d(testingSubstationAlignPos, Rotation2d.kZero));
    return pathfindingSubstationTargetPose;
  }

  public Zone getZone() {
    return zone;
  }

  public ScoringMode getScoringMode() {
    return scoringMode;
  }

  /** just returns the setpoint, see getSetpointWithUpdate() */
  public Setpoint getSetpoint() {
    return currentSetpoint;
  }

  /** updates the setpoint, then returns it, see getSetpoint() */
  public Setpoint getSetpointWithUpdate() {
    if (currentSetpoint != Setpoint.STOW) {
      if (scoringMode == ScoringMode.CUBE) {
        currentSetpoint = cubeSetpoints[scoringLevel];
      } else if (scoringMode == ScoringMode.CONE) {
        currentSetpoint = coneSetpoints[scoringLevel];
      }
    }

    return currentSetpoint;
  }

  public void updatePoses() {
    currentPose = Drive.getInstance().getPose();

    // determine pose of closest node depending on alliance
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get().equals(Alliance.Red)) {
      closestNodePose =
          new Pose2d(
              currentPose.getTranslation().nearest(Arrays.asList(Grids.redComplexLowTranslations)),
              Rotation2d.kZero);
    } else {
      closestNodePose =
          new Pose2d(
              currentPose.getTranslation().nearest(Arrays.asList(Grids.blueComplexLowTranslations)),
              Rotation2d.k180deg);
    }

    // TODO actually determine how substation logic works/find translations
    substationPose =
        new Pose2d(
            allianceFlip(
                new Translation2d(
                    LoadingZone.doubleSubstationX, LoadingZone.doubleSubstationCenterY)),
            Rotation2d.k180deg);

    // update zones
    scoringWallCorner = allianceFlip(FieldZones.wallCorner);
    scoringFeederCorner = allianceFlip(FieldZones.feederCorner);
    substationInnerCorner = allianceFlip(FieldZones.innerCorner);
    substationOuterCorner = allianceFlip(FieldZones.outerCorner);
  }

  public void updateZones() {
    Translation2d currentTranslation = currentPose.getTranslation();
    if (isWithinZone(scoringWallCorner, scoringFeederCorner, currentTranslation)) {
      zone = Zone.GRID;
    } else if (isWithinZone(substationOuterCorner, substationInnerCorner, currentTranslation)) {
      zone = Zone.SUBSTATION;
    } else {
      zone = Zone.NONE;
    }
  }

  /**
   * returns whether a given translation is within a rectangular box (defined by corner
   * translations)
   */
  public boolean isWithinZone(
      Translation2d corner1, Translation2d corner2, Translation2d currentTranslation) {
    double highX;
    double lowX;
    double highY;
    double lowY;
    if (corner1.getX() < corner2.getX()) {
      lowX = corner1.getX();
      highX = corner2.getX();
    } else {
      lowX = corner2.getX();
      highX = corner1.getX();
    }

    if (corner1.getY() < corner2.getY()) {
      lowY = corner1.getY();
      highY = corner2.getY();
    } else {
      lowY = corner2.getY();
      highY = corner1.getY();
    }

    return (currentTranslation.getX() > lowX && currentTranslation.getX() < highX)
        && (currentTranslation.getY() > lowY && currentTranslation.getY() < highY);
  }

  @Override
  public void periodic() {
    updatePoses();
    updateZones();
    Logger.recordOutput("State Manager/Setpoint", getSetpointWithUpdate());
    Logger.recordOutput("State Manager/Scoring Mode", scoringMode);
    Logger.recordOutput("Scoring Level", scoringLevel);
    Logger.recordOutput("State Manager/Nearest Node", getClosestNodePose());
    Logger.recordOutput("State Manager/Pathfinding Node Pose", pathfindingNodeTargetPose);
    Logger.recordOutput(
        "State Manager/Pathfinding Substation Pose", pathfindingSubstationTargetPose);
    Logger.recordOutput("State Manager/Zones/Current Zone", zone);
    Logger.recordOutput("State Manager/Zones/Scoring Wall Corner", scoringWallCorner);
    Logger.recordOutput("State Manager/Zones/Scoring Feeder Corner", scoringFeederCorner);
    Logger.recordOutput("State Manager/Zones/Substation Inner Corner", substationInnerCorner);
    Logger.recordOutput("State Manager/Zones/Substation Outer Corner", substationOuterCorner);
  }
}
