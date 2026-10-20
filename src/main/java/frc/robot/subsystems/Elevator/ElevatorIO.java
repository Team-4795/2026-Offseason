package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  // i don't know why the "AutoLog" and "ElevatorIOInputs" are mad
  @AutoLog
  public static class ElevatorIOInputs {
    public double ElevatorRightAppliedVolts = 0.0;
    public double elevatorRIghtPositionMeters = 0.0;
    public double elevatorRightVelocityMetersPerSecond = 0.0;
    public double elevatorRightCurrent = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs input) {}

  public default void setGoal(double goal) {}
}
