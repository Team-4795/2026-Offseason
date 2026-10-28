package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  // i don't know why the "AutoLog" and "ElevatorIOInputs" are mad
  @AutoLog
  public static class ElevatorIOInputs {
    public double goal = 0.0;
    public double setpointPos = 0.0;
    public double setpointVel = 0.0;

    public double elevatorRightAppliedVolts = 0.0;
    public double elevatorRightPositionMeters = 0.0;
    public double elevatorRightVelocityMetersPerSecond = 0.0;
    public double elevatorRightStatorCurrent = 0.0;
    public double elevatorRightSupplyCurrent = 0.0;

    public double elevatorLeftAppliedVolts = 0.0;
    public double elevatorLeftPositionMeters = 0.0;
    public double elevatorLeftVelocityMetersPerSecond = 0.0;
    public double elevatorLeftStatorCurrent = 0.0;
    public double elevatorLeftSupplyCurrent = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs input) {}

  public default void setGoal(double goal) {}
}
