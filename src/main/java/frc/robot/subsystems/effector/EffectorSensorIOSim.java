package frc.robot.subsystems.effector;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

public class EffectorSensorIOSim implements EffectorSensorIO {
  private boolean detected = false;
  private boolean hasGamepiece = false;

  private final Debouncer debouncer = new Debouncer(0.5);

  public EffectorSensorIOSim() {
    debouncer.setDebounceType(DebounceType.kBoth);
  }

  @Override
  public void toggleHasGamepiece() {
    detected = !detected;
  }

  @Override
  public boolean getHasGamepiece() {
    return hasGamepiece;
  }

  @Override
  public void updateInputs(EffectorSensorIOInputs inputs) {
    hasGamepiece = debouncer.calculate(detected);
    inputs.isDetected = detected;
    inputs.filteredIsDetected = hasGamepiece;
  }
}
