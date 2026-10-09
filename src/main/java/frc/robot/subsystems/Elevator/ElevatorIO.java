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

    public double elevatorLeftAppliedVolts = 0.0;
    public double elevatorLeftPositionMeters = 0.0;
    public double elevatorLeftVelocityMetersPerSecond = 0.0;
    public double elevatorLeftCurrent = 0.0;

    public ElevatorIOInputs(Object object) {
      // TODO Auto-generated constructor stub
    }

    public ElevatorIOInputs() {
      // TODO Auto-generated constructor stub
    }

    public Object setGoal(double rightX) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setGoal'");
    }
  }
}
