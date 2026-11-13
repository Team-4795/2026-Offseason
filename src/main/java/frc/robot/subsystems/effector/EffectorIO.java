package frc.robot.subsystems.effector;

import org.littletonrobotics.junction.AutoLog;

public interface EffectorIO {

  public default void setVoltage(double volts) {}

  public default void setGoal(double goal) {}

  public default boolean hasGamepiece() {
    return false;
  }

  @AutoLog
  public class EffectorIOInputs {
    public double position = 0;
    public double voltage = 0;
    public double velocity = 0;
    public double current = 0;

    public boolean hitThreshold = false;
    public boolean hasGamepiece = false;
  }

  public default void updateInputs(EffectorIOInputs inputs) {}
}
