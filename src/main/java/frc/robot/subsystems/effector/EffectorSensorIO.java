package frc.robot.subsystems.effector;

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
    return 6700.0;
  }

  // make it something like "return true if something is detected within x cm"
  public default boolean hasGamepiece() {
    return false;
  }

  public default void updateInputs(EffectorSensorIOInputs inputs) {}
}
