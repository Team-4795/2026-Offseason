package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PivotIOSim implements PivotIO {
  // IMPORTANT IMPORTANT:
  // you won't be able to use CTRE-specific stuff with the version of sim we're currently running
  // so you have to do the old-fashioned pid and ff controllers

  private DCMotorSim motor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.08, PivotConstants.GEARING),
          DCMotor.getKrakenX44(1));

  private TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          Units.rotationsToRadians(PivotConstants.MAX_V),
          Units.rotationsToRadians(PivotConstants.MAX_A));
  private TrapezoidProfile profile = new TrapezoidProfile(constraints);

  private PIDController pid =
      new PIDController(PivotConstants.SIM_kP, PivotConstants.SIM_kI, PivotConstants.SIM_kD);
  private ArmFeedforward feedFoward =
      new ArmFeedforward(
          PivotConstants.SIM_kS,
          PivotConstants.SIM_kG,
          PivotConstants.SIM_kV,
          PivotConstants.SIM_kA);

  private TrapezoidProfile.State goalState = new TrapezoidProfile.State(0.0, 0.0);
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State(0.0, 0.0);

  private double pidVolts;
  private double ffVolts;
  private double goalRads;

  public PivotIOSim() {}

  @Override
  public void setVoltage(double voltage) {
    motor.setInputVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  @Override
  public void setGoal(double goalRots) {
    goalRads =
        Units.rotationsToRadians(
            MathUtil.clamp(goalRots, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE));
    if (goalState.position != goalRads) {
      goalState = new TrapezoidProfile.State(goalRads, 0.0);
      setpointState =
          new TrapezoidProfile.State(
              motor.getAngularPositionRad(), motor.getAngularVelocityRadPerSec());
    }
  }

  @Override
  public double getPosition() {
    return motor.getAngularPositionRotations();
  }

  @Override
  public double getVelocity() {
    return motor.getAngularVelocityRPM() / 60.0;
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    setpointState = profile.calculate(0.02, setpointState, goalState);
    ffVolts = feedFoward.calculate(setpointState.position, setpointState.velocity);
    pidVolts = pid.calculate(motor.getAngularPositionRad(), setpointState.position);
    setVoltage(ffVolts + pidVolts);

    motor.update(0.02);

    inputs.velocity = getVelocity(); // so that it's all rps
    inputs.current = motor.getCurrentDrawAmps();
    inputs.goal = Units.radiansToRotations(goalState.position);
    inputs.position = getPosition();
    inputs.voltage = motor.getInputVoltage();
    inputs.setpointPos = Units.radiansToRotations(setpointState.position);
    inputs.setpointVel = Units.radiansToRotations(setpointState.velocity);
  }
}
