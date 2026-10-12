package frc.robot.subsystems.effector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class EffectorIOSim implements EffectorIO {
  private DCMotorSim effectorMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 2),
          DCMotor.getKrakenX60(1)); // should be neo vortex but also it doesnt really matter
  private double currentVoltage = 0;

  @Override
  public void setVoltage(double voltage) {
    currentVoltage = voltage;
    effectorMotor.setInputVoltage(voltage);
  }

  @Override
  public void updateInputs(EffectorIOInputs inputs) {
    inputs.position = effectorMotor.getAngularPositionRotations();
    inputs.velocity = effectorMotor.getAngularVelocityRPM() / 60; // rps
    inputs.voltage = currentVoltage;
    effectorMotor.update(0.02);
  }
}
