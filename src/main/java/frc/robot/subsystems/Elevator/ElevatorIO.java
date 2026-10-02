package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double voltage = 0;
    public double setpointPosition = 0;
    public double current = 0;
  }

  public default void setGoal(double v) {}

  public default double getPosition() {
    return 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}
}
