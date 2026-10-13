package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.stateManager.StateManager;

public class AutoCommands {
  private static final Drive drive = Drive.getInstance();
  private static final StateManager stateManager = StateManager.getInstance();

  public static Command autoAlignTest() {
    return new AutoAlignTest();
  }

  // ts does NOT work properly and im pretty sure its a pathplanner issue
  public static Command ppPathfindToNode() {
    Pose2d pose = stateManager.getPathfindingTargetPose();
    return DriveCommands.pathfindToPose(pose)
        // return DriveCommands.pathfindToPose(stateManager.pathfindDefaultPose)
        // .onlyWhile(() -> (stateManager.getNodeDistance() >
        // DriveConstants.pathfindHandoffDistance))
        .until(() -> stateManager.withinScoringZone(drive.getPose().getTranslation()))
        .andThen(autoAlignTest());
  }
}
