package frc.robot.subsystems.effector;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class EffectorIOReal implements EffectorIO {
  private SparkMax effectorMotor =
      new SparkMax(EffectorConstants.effectorSensorID, SparkLowLevel.MotorType.kBrushless);
  private double currentVoltage = 0;
  private RelativeEncoder effectorEncoder = effectorMotor.getEncoder();
  private final SparkMaxConfig config = new SparkMaxConfig();

  public EffectorIOReal() {
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(EffectorConstants.maxAmps);
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
