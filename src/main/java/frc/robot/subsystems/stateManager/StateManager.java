package frc.robot.subsystems.stateManager;

import static frc.robot.FieldConstants.*;

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
  private Pose2d currentPose;
  private Pose2d closestNodePose = new Pose2d();
  private Pose2d substationPose = new Pose2d();
  private Pose2d pathfindingNodeTargetPose;
  private Pose2d pathfindingSubstationTargetPose;

  private Translation2d scoringWallCorner = new Translation2d();
  private Translation2d scoringFeederCorner = new Translation2d();
  private Translation2d substationOuterCorner = new Translation2d();
  private Translation2d substationInnerCorner = new Translation2d();

  private Zone zone = Zone.NONE;
  private ScoringMode scoringMode = ScoringMode.NONE;

  private Setpoint[] coneSetpoints = {Setpoint.CONE_LOW, Setpoint.CONE_MID, Setpoint.CONE_HIGH};
  private Setpoint[] cubeSetpoints = {Setpoint.CUBE_LOW, Setpoint.CUBE_MID, Setpoint.CUBE_HIGH};

  private Setpoint currentSetpoint = Setpoint.STOW;

  // 0 - 2 inclusive, corresponding to setpoint array elements
  private int scoringLevel = 0;

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

  public void setScoringMode(ScoringMode m) {
    if (scoringMode != m) scoringMode = m;
  }

  public Setpoint getCurrentSetpoint() {
    return currentSetpoint;
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
    return pathfindingNodeTargetPose;
  }

  public Pose2d getSubstationPathfindingTargetPose() {
    return pathfindingSubstationTargetPose;
  }

  public Zone getZone() {
    return zone;
  }

  public ScoringMode getScoringMode() {
    return scoringMode;
  }

  public Setpoint updateAndGetSetpoint() {
    if (currentSetpoint != Setpoint.STOW) {
      if (scoringMode == ScoringMode.CUBE) {
        currentSetpoint = cubeSetpoints[scoringLevel];
      } else if (scoringMode == ScoringMode.CONE) {
        currentSetpoint = coneSetpoints[scoringLevel];
      }
    }

    return currentSetpoint;
  }

  public void setSetpointTesting() {
    currentSetpoint = Setpoint.CUBE_MID;
  }

  public void updatePoses() {
    currentPose = Drive.getInstance().getPose();
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get().equals(Alliance.Red)) {
      closestNodePose =
          new Pose2d(
              currentPose.getTranslation().nearest(Arrays.asList(Grids.redComplexLowTranslations)),
              Rotation2d.kZero);

      // TODO actually determine how substation logic works
      substationPose =
          new Pose2d(
              forceAllianceFlip(LoadingZone.singleSubstationTranslation), Rotation2d.k180deg);

      scoringWallCorner = forceAllianceFlip(FieldZones.wallCorner);
      scoringFeederCorner = forceAllianceFlip(FieldZones.feederCorner);
      substationInnerCorner = forceAllianceFlip(FieldZones.innerCorner);
      substationOuterCorner = forceAllianceFlip(FieldZones.outerCorner);

      pathfindingNodeTargetPose =
          new Pose2d(
              new Translation2d(
                  fieldLength - 1.2, MathUtil.clamp(currentPose.getTranslation().getY(), 0.5, 4.8)),
              Rotation2d.kZero);
      pathfindingSubstationTargetPose = new Pose2d(1.45, 6.75, Rotation2d.k180deg);
    } else {
      closestNodePose =
          new Pose2d(
              currentPose.getTranslation().nearest(Arrays.asList(Grids.complexLowTranslations)),
              Rotation2d.k180deg);

      // TODO this one too
      substationPose = new Pose2d(LoadingZone.singleSubstationTranslation, Rotation2d.kZero);

      scoringWallCorner = FieldZones.wallCorner;
      scoringFeederCorner = FieldZones.feederCorner;
      substationInnerCorner = FieldZones.innerCorner;
      substationOuterCorner = FieldZones.outerCorner;

      pathfindingNodeTargetPose =
          new Pose2d(
              new Translation2d(1.2, MathUtil.clamp(currentPose.getTranslation().getY(), 0.5, 4.8)),
              Rotation2d.k180deg);
      pathfindingSubstationTargetPose = new Pose2d(fieldLength - 1.45, 6.75, Rotation2d.kZero);
    }
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

    // if (DriverStation.getAlliance().isPresent()
    //     && DriverStation.getAlliance().get().equals(Alliance.Red)) {
    //   if ((currentTranslation.getY() > scoringFeederCorner.getY()
    //           && currentTranslation.getY() < scoringWallCorner.getY())
    //       && (currentTranslation.getX() < scoringWallCorner.getX()
    //           && currentTranslation.getX() > scoringFeederCorner.getX())) {
    //     zone = Zone.GRID;
    //   } else if ((currentTranslation.getY() > substationOuterCorner.getY()
    //           && currentTranslation.getY() < substationInnerCorner.getY())
    //       && (currentTranslation.getX() > substationInnerCorner.getX()
    //           && currentTranslation.getX() < substationOuterCorner.getX())) {
    //     zone = Zone.SUBSTATION;
    //   } else {
    //     zone = Zone.NONE;
    //   }
    // } else {
    //   if ((currentTranslation.getY() < scoringFeederCorner.getY()
    //           && currentTranslation.getY() > scoringWallCorner.getY())
    //       && (currentTranslation.getX() > scoringWallCorner.getX()
    //           && currentTranslation.getX() < scoringFeederCorner.getX())) {
    //     zone = Zone.GRID;
    //   } else if ((currentTranslation.getY() < substationInnerCorner.getY()
    //           && currentTranslation.getY() > substationOuterCorner.getY())
    //       && (currentTranslation.getX() > substationOuterCorner.getX()
    //           && currentTranslation.getX() < substationInnerCorner.getX())) {
    //     zone = Zone.SUBSTATION;
    //   } else {
    //     zone = Zone.NONE;
    //   }
    // }
  }

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
    Logger.recordOutput("State Manager/Setpoint", updateAndGetSetpoint());
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
