package frc.robot.subsystems.drive.effector;

public interface EffectorSensorIO {

  public default double getDistance() {
    return 6700.0;
  }
  ;
}
