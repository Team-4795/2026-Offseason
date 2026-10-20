package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.stateManager.StateManager;
import frc.robot.subsystems.stateManager.StateManager.Zone;
import java.util.function.DoubleSupplier;

public class AutoCommands {
  private static final Drive drive = Drive.getInstance();
  private static StateManager stateManager = StateManager.getInstance();

  public static Command autoAlignTest() {
    return new AutoAlign(drive);
  }

  // public static Command ppPathfindToPose() {
  //   return Commands.sequence(
  //     Commands.run(() -> {
  //       Pose2d targetPose = stateManager.getPathfindingTargetPose();
  //       DriveCommands.pathfindToPose(targetPose);
  //     }, drive)
  //   );
  //   // return DriveCommands.pathfindToPose(targetPose)
  //   //     .until(() -> stateManager.withinScoringZone(drive.getPose().getTranslation()))
  //   //     .andThen(autoAlignTest());
  // }

  public static Command ppPathfindToPose() {
    return DriveCommands.pathfindToPose(stateManager.getPathfindingTargetPose())
        .until(() -> stateManager.getZone() == Zone.GRID)
        .andThen(autoAlignTest());
  }

  public static Command ppPathfindWithOverride(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return DriveCommands.pathfindToPose(stateManager.getPathfindingTargetPose())
        .until(() -> stateManager.getZone() == Zone.GRID)
        .andThen(new AutoAlignWithOverride(drive, xSupplier, ySupplier));
  }
}
