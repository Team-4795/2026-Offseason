package frc.robot.subsystems.effector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Effector extends SubsystemBase {
  private EffectorIO io;
  private EffectorSensorIO sensorIo;

  // also include EffectorSensorIO as part of it

  private EffectorIOInputsAutoLogged inputs;
  private static Effector instance;

  public static Effector getInstance() {
    return instance;
  }

  public Effector(EffectorIO io, EffectorSensorIO sensorIo) {
    this.io = io;
    instance = this;
    this.sensorIo = sensorIo;
    inputs = new EffectorIOInputsAutoLogged();
  }

  public static Effector initialize(EffectorIO effectorIo, EffectorSensorIO sensorio) {
    if (instance == null) {
      instance = new Effector(effectorIo, sensorio);
    }
    return instance;
  }

  public void setEffectorVoltage(double volts) {
    io.setVoltage(volts);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }

  public void setGoal(double goal) {
    io.setGoal(goal);
  }

  // add setgoal
}
