package frc.robot.subsystems.effector;

import org.littletonrobotics.junction.AutoLog;

public interface EffectorIO {

  public default void setVoltage(double volts) {}

  @AutoLog
  public class effectorIOInputs {
    public double position = 0;
    public double voltage = 0;
    public double velocity = 0;
  }

  public default void updateInputs(effectorIOInputs inputs) {}
}
