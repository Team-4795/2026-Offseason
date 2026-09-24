package frc.robot.subsystems.effector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Effector extends SubsystemBase {
  private EffectorIO io;

  private static Effector instance;

  public static Effector getInstance() {
    return instance;
  }

  private Effector(EffectorIO io) {
    this.io = io;
    instance = this;
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

  // add setgoal
}
