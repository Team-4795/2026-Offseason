package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private PivotIO io;
  private static Pivot instance;
  private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private double goal = 0.0;

  private Pivot(PivotIO io) {
    this.io = io;
    instance = this;
    io.updateInputs(inputs);
  }

  public static Pivot getInstance() {
    return instance;
  }

  public static Pivot initialize(PivotIO io) {
    if (instance == null) instance = new Pivot(io);
    return instance;
  }

  public void setVoltage(double v) {
    io.setVoltage(v);
  }

  public void setGoal(double goalRots) {
    io.setGoal(MathUtil.clamp(goalRots, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE));
  }

  public boolean atGoal() {
    if (Math.abs(inputs.position - goal) < 0.3) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updateMotionProfile();
    Logger.processInputs("Pivot", inputs);
  }
}
