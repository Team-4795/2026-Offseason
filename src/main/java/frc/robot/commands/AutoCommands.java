package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.effector.Effector;
import frc.robot.subsystems.pivot.Pivot;

public class AutoCommands {
  private static Drive drive = Drive.getInstance();
  private static Effector effector = Effector.getInstance();
  private static Pivot pivot = Pivot.getInstance();

  public static Command intake() {
    return Commands.run(() -> effector.setEffectorVoltage(12), effector);
  }

  public static Command reverseIntake() {
    return Commands.run(() -> effector.setEffectorVoltage(-12), effector);
  }

  public static Command setPivotAngle(double angle) {
    return Commands.run(() -> pivot.setGoal(angle), pivot);
  }
}
