package frc.robot.subsystems.effector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Effector extends SubsystemBase {
  private EffectorIO effectorIo;
  // private EffectorSensorIO sensorIo;

  private EffectorIOInputsAutoLogged effectorInputs;
  // private EffectorSensorIOInputsAutoLogged sensorInputs;
  private static Effector instance;

  public static Effector getInstance() {
    return instance;
  }

  private Effector(EffectorIO io) {
    this.effectorIo = io;
    // this.sensorIo = sensorIo;
    instance = this;
    effectorInputs = new EffectorIOInputsAutoLogged();
    // sensorInputs = new EffectorSensorIOInputsAutoLogged();
  }

  public static Effector initialize(EffectorIO effectorIo) {
    if (instance == null) {
      instance = new Effector(effectorIo);
    }
    return instance;
  }

  public void setEffectorVoltage(double volts) {
    effectorIo.setVoltage(volts);
  }

  public void setGoal(double goal) {
    effectorIo.setGoal(goal);
  }

  public boolean hasGamepiece() {
    // return sensorIo.getHasGamepiece();
    return effectorIo.hasGamepiece();
  }

  // public double getDistance() {
  //   return sensorIo.getDistance();
  // }

  /** only used when simulating effector */
  // public void debugToggleHasGamepiece() {
  //   sensorIo.toggleHasGamepiece();
  // }

  public void defaultCommand() {
    if (hasGamepiece()) setEffectorVoltage(3);
    else setEffectorVoltage(0); // TODO do we want to keep intake constantly running?
  }

  @Override
  public void periodic() {
    effectorIo.updateInputs(effectorInputs);
    Logger.processInputs("Effector", effectorInputs);
    // sensorIo.updateInputs(sensorInputs);
    // Logger.processInputs("Effector/Sensor", sensorInputs);
  }
}
