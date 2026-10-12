package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  //IMPLEMENT IOINPUTS
  private static ElevatorIO elevatorIO;
  private static ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();
  public Elevator(ElevatorIO io)
  {
    elevatorIO = io;
  }
  public void setGoal(double rightX) {
    elevatorIO.setGoal(rightX);
  }
}
