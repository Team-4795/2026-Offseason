package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public class PivotIOInputs {
    public double position = 0;
    public double velocity = 0;
    public double voltage = 0;
    public double current = 0;
    public double goal = 0;
    public double setpointPos = 0;
    public double setpointVel = 0;
  }

  public default void setVoltage(double v) {}

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void initialize() {}

  public default void setGoal(double goalRots) {}

  public default double getPosition() {
    return 0;
  }

  public default double getVelocity() {
    return 0;
  }

  public default void updateMotionProfile() {}
}
