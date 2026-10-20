package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private VisionIO[] io;

  public static Vision instance;

  public static Vision getInstance() {
    return instance;
  }
}
