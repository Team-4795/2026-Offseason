package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  // i don't know why the "AutoLog" and "ElevatorIOInputs" are mad
  @AutoLog
  public static class ElevatorIOInputs {
    public double goal = 0.0;
    public double setpointPos = 0.0;
    public double setpointVel = 0.0;

    public double rightAppliedVolts = 0.0;
    public double rightPositionMeters = 0.0;
    public double rightVelocityMetersPerSecond = 0.0;
    public double rightStatorCurrent = 0.0;
    public double rightSupplyCurrent = 0.0;

    public double leftAppliedVolts = 0.0;
    public double leftPositionMeters = 0.0;
    public double leftVelocityMetersPerSecond = 0.0;
    public double leftStatorCurrent = 0.0;
    public double leftSupplyCurrent = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs input) {}

  public default void setGoal(double goal) {}

  public default double getGoal() {
    return 0.0;
  }
}
