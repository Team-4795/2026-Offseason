package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private static StateManager stateManager = StateManager.getInstance();

  public static Command intake() {
    return Commands.run(() -> effector.setEffectorVoltage(12), effector);
  }

  public static Command reverseIntake() {
    return Commands.run(() -> effector.setEffectorVoltage(-12), effector);
  }

  public static Command setPivotAngle(double angle) {
    return Commands.run(() -> pivot.setGoal(angle), pivot);
  }

  public static Command autoAlignTest() {
    return new AutoAlignNode(drive);
  }

  public static Command ppPathfindToPose() {
    return DriveCommands.pathfindToPose(stateManager.getNodePathfindingTargetPose())
        .until(() -> stateManager.getZone() == Zone.GRID)
        .andThen(autoAlignTest());
  }

  public static Command pathfindNodeWithOverride(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return DriveCommands.pathfindToPose(stateManager.getNodePathfindingTargetPose())
        .until(() -> stateManager.getZone() == Zone.GRID)
        .andThen(new AlignNodeWithOverride(drive, xSupplier, ySupplier));
  }

  public static Command pathfindSubstationWithOverride(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return DriveCommands.pathfindToPose(stateManager.getSubstationPathfindingTargetPose())
        .until(() -> stateManager.getZone() == Zone.SUBSTATION)
        .andThen(new AlignSubstationWithOverride(drive, xSupplier, ySupplier));
  }

  public static Command setCubeMode() {
    return Commands.runOnce(() -> stateManager.setScoringMode(ScoringMode.CUBE));
  }

  public static Command setConeMode() {
    return Commands.runOnce(() -> stateManager.setScoringMode(ScoringMode.CONE));
  }
}
