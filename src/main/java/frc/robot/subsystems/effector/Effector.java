package frc.robot.subsystems.effector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Effector extends SubsystemBase {
  private EffectorIO io;

  // also include EffectorSensorIO as part of it

  private EffectorIOInputsAutoLogged inputs;
  private static Effector instance;

  public static Effector getInstance() {
    return instance;
  }

  public Effector(EffectorIO io) {
    this.io = io;
    instance = this;
    inputs = new EffectorIOInputsAutoLogged();
  }

  public static Effector initialize(EffectorIO effectorIo) {
    if (instance == null) {
      instance = new Effector(effectorIo);
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

  // add setgoal
}
