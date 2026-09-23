package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public class ElevatorIOInputs {
    public Rotation2d position = new Rotation2d(0);
    public double velocity = 0;
    public double voltage = 0;
    public double current = 0;
    public Rotation2d goal = new Rotation2d(0);
  }

  public default void setVoltage(double v) {}
  ;

  public default void updateInputs(ElevatorIOInputs inputs) {}
  ;

  public default void setGoal(Rotation2d goal) {}
  ;

  public default Rotation2d getPosition() {
    return new Rotation2d();
  }
  ;
}
