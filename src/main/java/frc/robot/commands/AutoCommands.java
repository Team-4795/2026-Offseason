package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.stateManager.StateManager;

public class AutoCommands {
  private static final Drive drive = Drive.getInstance();
  private static final StateManager stateManager = StateManager.getInstance();

  public static Command autoAlignTest() {
    return new AutoAlignTest();
  }

  // TODO: pathfind to general area, then auto align for precise lineup?
  public static Command ppPathfindToNode() {
    return Commands.run(() -> drive.pathfindToPose(stateManager.getClosestNodePose()))
        .onlyWhile(() -> (stateManager.getNodeDistance() > DriveConstants.pathfindHandoffDistance))
        .andThen(autoAlignTest());
  }
}
