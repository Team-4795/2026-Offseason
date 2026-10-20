package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // IMPLEMENT IOINPUTS
  private static ElevatorIO elevatorIO;
  private static ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    elevatorIO = io;
  }

  public void setGoal(double rightX) {
    elevatorIO.setGoal(rightX);
  }

  @Override
  public void periodic() {
    elevatorIO.updateInputs(inputs);
    Logger.processInputs("elevator", inputs);
  }
}
