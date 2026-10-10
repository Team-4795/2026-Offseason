package frc.robot.subsystems.effector;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.effector.EffectorConstants.SensorConstants;

public class EffectorSensorIOReal implements EffectorSensorIO {
  private final CANrange sensor = new CANrange(SensorConstants.SENSOR_ID);
  private final CANrangeConfiguration config = new CANrangeConfiguration();

  private final StatusSignal<Distance> distance = sensor.getDistance();
  private final StatusSignal<Boolean> isDetected = sensor.getIsDetected();

  public EffectorSensorIOReal() {
    // we need to configure this thing to act as a beam-break sensor, do some research on that
    config.FovParams.FOVRangeX = SensorConstants.FOV_X;
    config.FovParams.FOVRangeY = SensorConstants.FOV_Y;
    config.ProximityParams.ProximityThreshold = SensorConstants.PROXIMITY_THRESHOLD;
    sensor.getConfigurator().apply(config);
  }

  @Override
  public double getDistance() {
    return distance.refresh().getValue().in(Units.Centimeters);
  }

  @Override
  public boolean hasGamepiece() {
    return isDetected.refresh().getValue();
  }
}
