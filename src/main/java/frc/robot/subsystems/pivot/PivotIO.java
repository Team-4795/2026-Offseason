package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public class PivotIOInputs {
    public double position = 0;
    public double velocity = 0;
    public double voltage = 0;
    public double current = 0;
    public double goal = 0;
  }

  public default void setVoltage(double v) {}

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void initialize() {}

  public default void setGoal(double goal) {}

  public default void updateMotionProfile() {}

  public default Rotation2d getPosition() {
    return Rotation2d.kZero;
  }
  ;
}
