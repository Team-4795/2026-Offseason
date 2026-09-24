package frc.robot.subsystems.effector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Effector extends SubsystemBase {
  private EffectorIO effectorIo;

  private static Effector instance;

  public static Effector getInstance() {
    return instance;
  }

  public Effector(EffectorIO io) {
    effectorIo = io;
    instance = this;
  }

  public static Effector Initialize(EffectorIO effectorIo) {
    if (instance == null) {
      instance = new Effector(effectorIo);
    }
    return instance;
  }

  public void setEffectorVoltage(double volts) {
    effectorIo.setVoltage(volts);
  }
}
