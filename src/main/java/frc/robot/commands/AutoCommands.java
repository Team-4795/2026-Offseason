package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.effector.Effector;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.stateManager.StateManager;
import frc.robot.subsystems.stateManager.StateManager.ScoringMode;
import frc.robot.subsystems.stateManager.StateManager.Zone;
import java.util.function.DoubleSupplier;

public class AutoCommands {
  private static Drive drive = Drive.getInstance();
  private static Effector effector = Effector.getInstance();
  private static Pivot pivot = Pivot.getInstance();
  private static Elevator elevator = Elevator.getInstance();
  private static StateManager stateManager = StateManager.getInstance();

  public static Command runEffector() {
    return Commands.run(() -> effector.setEffectorVoltage(10), effector);
  }

  public static Command reverseEffector() {
    return Commands.run(() -> effector.setEffectorVoltage(-6), effector);
  }

  public static Command intake() {
    return runEffector().onlyWhile(() -> !effector.hasGamepiece());
  }

  public static Command outtake() {
    return Commands.runEnd(
        () -> effector.setEffectorVoltage(-6),
        () -> stateManager.setScoringMode(ScoringMode.NONE),
        effector);
  }

  public static Command setPivotAngle(double angleRots) {
    return Commands.runOnce(() -> pivot.setGoal(angleRots), pivot);
  }

  public static Command setElevatorPos(double goal) {
    return Commands.runOnce(() -> elevator.setGoal(goal), elevator);
  }

  /** Set pivot goal to state manager setpoint */
  public static Command pivotToSetpoint() {
    return Commands.runOnce(() -> pivot.setGoal(stateManager.getSetpoint().pivotPos), pivot);
  }

  /** Set elevator goal to state manager setpoint */
  public static Command elevatorToSetpoint() {
    return Commands.runOnce(
        () -> elevator.setGoal(stateManager.getSetpoint().elevatorPos), elevator);
  }

  // public static Command ppPathfindToPose() {
  //   return DriveCommands.pathfindToPose(stateManager.getNodePathfindingTargetPose())
  //       .until(() -> stateManager.getZone() == Zone.GRID)
  //       .andThen(autoAlignTest());
  // }

  // public static Command autoAlignTest() {
  //   return new AutoAlignNode(drive);
  // }

  /**
   * Use Pathplanner to pathfind roughly to closest node, then auto align (with driver joystick
   * override)
   */
  public static Command pathfindNodeWithOverride(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return DriveCommands.pathfindToPose(stateManager.getNodePathfindingTargetPose())
        .until(() -> stateManager.getZone() == Zone.GRID)
        .andThen(new AlignNodeWithOverride(drive, xSupplier, ySupplier));
  }

  /**
   * Use Pathplanner to pathfind roughly to substation, then auto align (with driver joystick
   * override)
   */
  public static Command pathfindSubstationWithOverride(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return DriveCommands.pathfindToPose(stateManager.getSubstationPathfindingTargetPose())
        .until(() -> stateManager.getZone() == Zone.SUBSTATION)
        .andThen(new AlignSubstationWithOverride(drive, xSupplier, ySupplier));
  }

  /** Toggle cube mode for scoring (see {@link #setConeMode()}) */
  public static Command setCubeMode() {
    return Commands.runOnce(() -> stateManager.setScoringMode(ScoringMode.CUBE));
  }

  /** Toggle cone mode for scoring (see {@link #setCubeMode()}) */
  public static Command setConeMode() {
    return Commands.runOnce(() -> stateManager.setScoringMode(ScoringMode.CONE));
  }

  public static Command scoringSequence() {
    return Commands.sequence(
        Commands.parallel(pivotToSetpoint(), elevatorToSetpoint()),
        Commands.waitUntil(() -> (pivot.atGoal() && elevator.atGoal())),
        outtake());
  }
}
