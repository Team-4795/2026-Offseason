package frc.robot.subsystems.effector;

public interface EffectorSensorIO {

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
}
