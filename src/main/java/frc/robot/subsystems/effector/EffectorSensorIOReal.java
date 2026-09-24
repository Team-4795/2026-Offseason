package frc.robot.subsystems.effector;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.effector.EffectorConstants.SensorConstants;

public class EffectorSensorIOReal implements EffectorSensorIO {
  private final CANrange sensor = new CANrange(SensorConstants.SENSOR_ID);
  CANrangeConfiguration config = new CANrangeConfiguration();

  public
  EffectorSensorIOReal() { 
    // we need to configure this thing to act as a beam-break sensor, do some research on that
    config.FovParams.FOVRangeX = SensorConstants.FOV_X;
    config.FovParams.FOVRangeY = SensorConstants.FOV_Y;
    config.ProximityParams.ProximityThreshold = SensorConstants.PROXIMITY_THRESHOLD;
    sensor.getConfigurator().apply(config);
  }

  @Override
  public double getDistance() {
    return sensor.getDistance().refresh().getValue().in(Units.Centimeters);
  }
}
