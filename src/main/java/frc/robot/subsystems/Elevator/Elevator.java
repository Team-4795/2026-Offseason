package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // IMPLEMENT IOINPUTS
  private static ElevatorIO elevatorIO;
  private static ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private static Elevator instance;

  public static Elevator initialize(ElevatorIO io) {
    if (instance == null) instance = new Elevator(io);
    return instance;
  }

  private Elevator(ElevatorIO io) {
    elevatorIO = io;
  }

  public void setGoal(double goal) {
    elevatorIO.setGoal(
        MathUtil.clamp(goal, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT));
  }

  public boolean atGoal() {
    if (Math.abs(inputs.rightPositionMeters - elevatorIO.getGoal()) < 0.2) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    elevatorIO.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public static Elevator getInstance() {
    return instance;
  }
}
