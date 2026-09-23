package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private PivotIO io;
  private static Pivot instance;
  private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  public enum Setpoints {
    S1,
    S2,
    S3,
  }

  public Pivot(PivotIO eIo) {
    this.io = eIo;
    instance = this;
    io.updateInputs(inputs);
  }

  public static Pivot getInstance() {
    return instance;
  }

  public static Pivot Initialize(PivotIO io) {
    if (instance == null) instance = new Pivot(io);
    return instance;
  }

  public void setVoltage(double v) {
    io.setVoltage(v);
  }

  public void setGoal(Rotation2d goal) {
    io.setGoal(goal);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
  }
}
