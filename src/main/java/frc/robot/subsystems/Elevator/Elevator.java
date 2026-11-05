package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // IMPLEMENT IOINPUTS
  private static ElevatorIO elevatorIO;
  private static ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private static Elevator instance;
  private double goal = 0.0;

  public Elevator(ElevatorIO io) {
    elevatorIO = io;
  }

  public void setGoal(double rightX) {
    elevatorIO.setGoal(rightX);
    goal = rightX;
  }

  public boolean atGoal() {
    if (Math.abs(inputs.elevatorRightPositionMeters - goal) < 0.2) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    elevatorIO.updateInputs(inputs);
    Logger.processInputs("elevator", inputs);
  }

  public static Elevator getInstance() {
    return instance;
  }
}
