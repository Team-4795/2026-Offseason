package frc.robot.subsystems.effector.sensor;

import org.littletonrobotics.junction.AutoLog;

public interface EffectorSensorIO {
  @AutoLog
  public class EffectorSensorIOInputs {
    public double distance;
    public boolean isDetected;
    public boolean filteredIsDetected;
  }

  /**
   * @return distance of sensor, in centimeters
   */
  public default double getDistance() {
    return 0.0;
  }

  public default boolean getHasGamepiece() {
    return false;
  }

  public default void toggleHasGamepiece() {}

  public default void updateInputs(EffectorSensorIOInputs inputs) {}
}
