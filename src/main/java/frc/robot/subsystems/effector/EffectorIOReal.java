package frc.robot.subsystems.effector;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkMaxConfig;

public class EffectorIOReal implements EffectorIO {
  private SparkFlex effectorMotor =
      new SparkFlex(EffectorConstants.CAN_ID, SparkLowLevel.MotorType.kBrushless);
  private double currentVoltage = 0;
  private RelativeEncoder effectorEncoder = effectorMotor.getEncoder();
  private final SparkMaxConfig config = new SparkMaxConfig();

  public EffectorIOReal() {
    config.smartCurrentLimit(EffectorConstants.CURRENT_LIMIT);
    config.voltageCompensation(12.0);

    effectorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setVoltage(double voltage) {
    effectorMotor.setVoltage(voltage);
    currentVoltage = voltage;
  }

  @Override
  public void updateInputs(EffectorIOInputs inputs) {
    inputs.position = effectorEncoder.getPosition();
    inputs.voltage = currentVoltage;
    inputs.velocity = effectorEncoder.getVelocity() / 60.0; // rps
    inputs.current = effectorMotor.getOutputCurrent(); // sonion...
  }
}
