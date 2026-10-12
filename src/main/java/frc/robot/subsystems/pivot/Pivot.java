package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private PivotIO io;
  private static Pivot instance;
  private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private LoggedTunableNumber kp =
      new LoggedTunableNumber("Pivot/Kp2", PivotConstants.kP);
  private LoggedTunableNumber ki =
      new LoggedTunableNumber("Pivot/Ki2", PivotConstants.kI);
  private LoggedTunableNumber kd =
      new LoggedTunableNumber("Pivot/Kd2", PivotConstants.kD);

  private LoggedTunableNumber ks =
      new LoggedTunableNumber("Pivot/Ks2", PivotConstants.kS);
  private LoggedTunableNumber kv =
      new LoggedTunableNumber("Pivot/Kv2", PivotConstants.kV);
  private LoggedTunableNumber ka =
      new LoggedTunableNumber("Pivot/Ka2", PivotConstants.kA);
  private LoggedTunableNumber kg =
      new LoggedTunableNumber("Pivot/Kg2", PivotConstants.kG);

  // you don't need to worry about setpoints right now, we will make a state manager that handles
  // stuff like this
  public enum Setpoints {
    S1,
    S2,
    S3,
  }

  private Pivot(PivotIO io) { // private because we really shouldn't be using it
    this.io = io;
    instance = this;
    io.configure(kp.getAsDouble(),
        ki.getAsDouble(),
        kd.getAsDouble(),
        ks.getAsDouble(),
        kv.getAsDouble(),
        ka.getAsDouble(),
        kg.getAsDouble());
  }
  public void Configure(){
    io.configure(0, 0, 0, 0, 0, 0, 0);
  }
  public static Pivot getInstance() {
    return instance;
  }

  public static Pivot initialize(PivotIO io) {
    if (instance == null) instance = new Pivot(io);
    return instance;
  }

  public void setVoltage(double v) {
    io.setVoltage(v);
  }

  public void setGoal(double goalRots) {
    io.setGoal(goalRots);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updateMotionProfile();
    Logger.processInputs("Pivot", inputs);
  }
}
