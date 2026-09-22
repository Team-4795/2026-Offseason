package frc.robot.subsystems.drive.effector;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.Units;

public class EffectorSensorIOReal implements EffectorSensorIO {
  private final CANrange sensor;

  public EffectorSensorIOReal() {
    sensor = new CANrange(EffectorConstants.effectorSensorID);
    CANrangeConfiguration config = new CANrangeConfiguration();
    sensor.getConfigurator().apply(config);
  }

  @Override
  public double getDistance() {
    return sensor.getDistance().refresh().getValue().in(Units.Centimeters);
  }
}
