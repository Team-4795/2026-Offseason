package frc.robot.subsystems.effector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.effector.EffectorConstants.SensorConstants;

public class EffectorSensorIOReal implements EffectorSensorIO {
  private final CANrange sensor = new CANrange(SensorConstants.SENSOR_ID);
  private final CANrangeConfiguration config = new CANrangeConfiguration();

  private final StatusSignal<Distance> distance = sensor.getDistance();
  private final StatusSignal<Boolean> isDetected = sensor.getIsDetected();

  private final Debouncer filter = new Debouncer(0.5); // unless canrange has tihs thing already

  private boolean filteredIsDetected;

  public EffectorSensorIOReal() {
    BaseStatusSignal.setUpdateFrequencyForAll(50, distance, isDetected);

    // we need to configure this thing to act as a beam-break sensor, do some research on that
    config.FovParams.FOVRangeX = SensorConstants.FOV_X;
    config.FovParams.FOVRangeY = SensorConstants.FOV_Y;
    config.ProximityParams.ProximityThreshold = SensorConstants.PROXIMITY_THRESHOLD;
    config.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
    sensor.getConfigurator().apply(config);
  }

  @Override
  public double getDistance() {
    return distance.getValue().in(Units.Centimeters);
  }

  @Override
  public boolean hasGamepiece() {
    return filter.calculate(isDetected.getValue());
  }

  @Override
  public void updateInputs(EffectorSensorIOInputs inputs) {
    inputs.distance = distance.getValue().in(Units.Centimeters);
    inputs.isDetected = isDetected.getValue();
    inputs.filteredIsDetected = filteredIsDetected;
  }
}
