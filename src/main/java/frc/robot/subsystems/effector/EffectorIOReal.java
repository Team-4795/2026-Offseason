package frc.robot.subsystems.effector;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.effector.EffectorConstants.SensorConstants;

public class EffectorIOReal implements EffectorIO {
  // don't use onboard pid for sparks
  // do we even need pid?
  private SparkMax effectorMotor =
      new SparkMax(SensorConstants.SENSOR_ID, SparkLowLevel.MotorType.kBrushless);
  private double currentVoltage = 0;
  private RelativeEncoder effectorEncoder = effectorMotor.getEncoder();
  private final SparkMaxConfig config = new SparkMaxConfig();

  public EffectorIOReal() {
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(EffectorConstants.CURRENT_LIMIT);

    // don't forget to apply the config
  }

  @Override
  public void setVoltage(double voltage) {
    effectorMotor.setVoltage(voltage);
    currentVoltage = voltage;
  }

  @Override
  public void updateInputs(effectorIOInputs inputs) {
    inputs.position = effectorEncoder.getPosition();
    inputs.voltage = currentVoltage;
    inputs.velocity = effectorEncoder.getVelocity();
  }
}
