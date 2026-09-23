package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private static Elevator instance;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public enum Setpoints {
    S1,
    S2,
    S3,
  }

  public Elevator(ElevatorIO eIo) {
    this.io = eIo;
    instance = this;
    io.updateInputs(inputs);
  }

  public static Elevator getInstance() {
    return instance;
  }

  public static Elevator Initialize(ElevatorIO io) {
    if (instance == null) instance = new Elevator(io);
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
    Logger.processInputs("elevator", inputs);
  }
}
