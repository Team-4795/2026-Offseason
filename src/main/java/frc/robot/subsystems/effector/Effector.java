package frc.robot.subsystems.effector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Effector extends SubsystemBase {
  private EffectorIO effectorIo;
  private EffectorSensorIO sensorIo;

  private EffectorIOInputsAutoLogged effectorInputs;
  private EffectorSensorIOInputsAutoLogged sensorInputs;
  private static Effector instance;

  public static Effector getInstance() {
    return instance;
  }

  public Effector(EffectorIO io, EffectorSensorIO sensorIo) {
    this.effectorIo = io;
    instance = this;
    this.sensorIo = sensorIo;
    effectorInputs = new EffectorIOInputsAutoLogged();
  }

  public static Effector initialize(EffectorIO effectorIo, EffectorSensorIO sensorio) {
    if (instance == null) {
      instance = new Effector(effectorIo, sensorio);
    }
    return instance;
  }

  public void setEffectorVoltage(double volts) {
    effectorIo.setVoltage(volts);
  }

  public void setGoal(double goal) {
    effectorIo.setGoal(goal);
  }

  @Override
  public void periodic() {
    effectorIo.updateInputs(effectorInputs);
    Logger.processInputs("Effector", effectorInputs);
    sensorIo.updateInputs(sensorInputs);
    Logger.processInputs("Effector/Sensor", sensorInputs);
  }
}
