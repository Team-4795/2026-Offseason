package frc.robot.subsystems.effector;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.stateManager.StateManager;
import frc.robot.subsystems.stateManager.StateManager.State;

public class EffectorIOSim implements EffectorIO {
  private DCMotorSim effectorMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 2),
          DCMotor.getKrakenX60(1)); // should be neo vortex but also it doesnt really matter
  private double currentVoltage = 0;

  private boolean hasGamepiece = false;

  private boolean shouldSpikeCurrent = false;
  private boolean shouldLowerVelocity = false;

  private Debouncer debouncer =
      new Debouncer(EffectorConstants.DEBOUNCE_TIME, DebounceType.kRising);

  @Override
  public void setVoltage(double voltage) {
    currentVoltage = voltage;
    effectorMotor.setInputVoltage(voltage);
  }

  @Override
  public boolean hasGamepiece() {
    return hasGamepiece;
  }

  @Override
  public void debugLowerVelocity() {
    shouldLowerVelocity = !shouldLowerVelocity;
  }

  @Override
  public void debugSpikeCurrent() {
    shouldSpikeCurrent = !shouldSpikeCurrent;
  }

  @Override
  public void updateInputs(EffectorIOInputs inputs) {
    effectorMotor.update(0.02);
    inputs.position = effectorMotor.getAngularPositionRotations();
    inputs.velocity = effectorMotor.getAngularVelocityRPM() / 60.0; // rps
    inputs.voltage = currentVoltage;
    inputs.current = effectorMotor.getCurrentDrawAmps();

    if (StateManager.getInstance().getState() == State.INTAKING) {
      if (EffectorConstants.VELOCITY_MODE) {
        inputs.hitThreshold = shouldLowerVelocity;
        hasGamepiece = debouncer.calculate(inputs.hitThreshold);
      } else {
        inputs.hitThreshold = shouldSpikeCurrent;
        // hasGamepiece = debouncer.calculate(inputs.hitThreshold); // spike vs. constant current
        hasGamepiece = inputs.hitThreshold;
      }
    }
  }
}
