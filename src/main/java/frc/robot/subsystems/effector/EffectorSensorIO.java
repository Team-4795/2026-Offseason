package frc.robot.subsystems.effector;

public interface EffectorSensorIO {

  /**
   * @return distance of sensor, in centimeters
   */
  public default double getDistance() {
    return 6700.0;
  }
  ;
}
