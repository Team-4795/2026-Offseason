package frc.robot.subsystems.effector;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

public class EffectorIOReal implements EffectorIO {
  private SparkFlex effectorMotor =
      new SparkFlex(EffectorConstants.CAN_ID, SparkLowLevel.MotorType.kBrushless);
  private double currentVoltage = 0;
  private RelativeEncoder effectorEncoder = effectorMotor.getEncoder();
  private final SparkMaxConfig config = new SparkMaxConfig();

  private double velocity = 0.0;
  private double current = 0.0;

  private Debouncer debouncer = new Debouncer(EffectorConstants.DEBOUNCE_TIME);
  private boolean hasGamepiece = false;

  public EffectorIOReal() {
    debouncer.setDebounceType(DebounceType.kBoth);

    config.smartCurrentLimit(
        EffectorConstants.STALL_CURRENT_LIMIT,
        EffectorConstants.FREE_CURRENT_LIMIT,
        (int) EffectorConstants.VELOCITY_THRESHOLD);
    config.voltageCompensation(12.0);
    config.inverted(true);

    effectorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setVoltage(double voltage) {
    effectorMotor.setVoltage(voltage);
    currentVoltage = voltage;
  }

  @Override
  public boolean hasGamepiece() {
    return hasGamepiece;
  }

  @Override
  public void updateInputs(EffectorIOInputs inputs) {
    inputs.position = effectorEncoder.getPosition();
    inputs.voltage = currentVoltage;

    velocity = effectorEncoder.getVelocity() / 60.0; // rps
    inputs.velocity = velocity;

    current = effectorMotor.getOutputCurrent();
    inputs.current = current;

    if (EffectorConstants.VELOCITY_MODE) {
      inputs.hitThreshold = velocity < EffectorConstants.VELOCITY_THRESHOLD;
      hasGamepiece = debouncer.calculate(inputs.hitThreshold);
    } else {
      inputs.hitThreshold = current > EffectorConstants.CURRENT_THRESHOLD;
      // hasGamepiece = debouncer.calculate(inputs.hitThreshold); // spike vs. constant current
      hasGamepiece = inputs.hitThreshold;
    }

    inputs.hasGamepiece = hasGamepiece;
  }
}
