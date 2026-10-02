package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private static Elevator instance;
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public static Elevator initialize(ElevatorIO io) {
    if (instance == null) {
      instance = new Elevator(io);
    }
    return instance;
  }

  public Elevator(ElevatorIO i) {
    this.io = i;
    io.updateInputs(inputs);
  }

  public static Elevator getInstance() {
    return instance;
  }

  public void setGoal(double g) {
    io.setGoal(g);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }
}
