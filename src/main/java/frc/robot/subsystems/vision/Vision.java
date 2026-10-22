package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private VisionIO[] io;
  private VisionIOInputsAutoLogged[] inputs;

  public static Vision instance;

  public static Vision getInstance() {
    return instance;
  }
  
  public static Vision initilize(VisionIO... io) {
    if (instance == null) {
      instance = new Vision(io);
    }
    return instance;
  }

  public Vision(VisionIO visionIO[]) {
    io = visionIO;
    inputs = new VisionIOInputsAutoLogged[io.length];

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }
}
