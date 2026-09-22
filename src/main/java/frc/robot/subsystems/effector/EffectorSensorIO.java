package frc.robot.subsystems.effector;

public interface EffectorSensorIO {

  public default double getDistance() {
    return 6700.0;
  }
  ;
}
