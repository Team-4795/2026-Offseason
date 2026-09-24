package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private PivotIO io;
  private static Pivot instance;
  private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  // you don't need to worry about setpoints right now, we will make a state manager that handles
  // stuff like this
  public enum Setpoints {
    S1,
    S2,
    S3,
  }

  private Pivot(PivotIO io) { // private because we really shouldn't be using it
    this.io = io;
    instance = this;
    io.updateInputs(inputs);
  }

  public static Pivot getInstance() {
    return instance;
  }

  public static Pivot initialize(PivotIO io) {
    if (instance == null) instance = new Pivot(io);
    return instance;
  }

  public void setVoltage(double v) {
    io.setVoltage(v);
  }

  public void setGoal(double goal) {
    io.setGoal(goal);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updateMotionProfile();
    Logger.processInputs("Pivot", inputs);
  }
}
