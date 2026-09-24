package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), 0.0001, PivotConstants.GEARING),
          DCMotor.getKrakenX60(1));
  private TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(PivotConstants.MAX_V, PivotConstants.MAX_A);
  private TrapezoidProfile profile = new TrapezoidProfile(constraints);
  private ProfiledPIDController pid =
      new ProfiledPIDController(
          PivotConstants.SIM_kP, PivotConstants.SIM_kI, PivotConstants.SIM_kD, constraints);
  private SimpleMotorFeedforward feedFoward =
      new SimpleMotorFeedforward(
          PivotConstants.SIM_kS, PivotConstants.SIM_kV, PivotConstants.SIM_kA);

  private TrapezoidProfile.State goalState = new TrapezoidProfile.State(0.0, 0.0);
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State(0.0, 0.0);

  private double clampedGoal;
  private double pidVolts;
  private double ffVolts;

  public PivotIOSim() {
    // configs.Slot0.kA = PivotConstants.kA;
    // configs.Slot0.kG = PivotConstants.kG;
    // configs.Slot0.kS = PivotConstants.kS;
    // configs.Slot0.kV = PivotConstants.kV;
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setInputVoltage(voltage);
  }

  @Override
  public void setGoal(double goalRots) {
    if (goalState.position != goalRots) {
      clampedGoal = MathUtil.clamp(goalRots, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE);
      goalState = new TrapezoidProfile.State(Units.rotationsToRadians(clampedGoal), 0.0);
      setpointState =
          new TrapezoidProfile.State(
              motor.getAngularPositionRad(), motor.getAngularVelocityRadPerSec());
    }
  }

  @Override
  public void updateMotionProfile() {
    setpointState = profile.calculate(0.02, setpointState, goalState);
    ffVolts = feedFoward.calculate(setpointState.velocity);
    pidVolts = pid.calculate(motor.getAngularPositionRotations(), setpointState.position);
    setVoltage(ffVolts + pidVolts);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.velocity = motor.getAngularVelocityRPM() * 60; // so that it's all rps
    inputs.current = motor.getCurrentDrawAmps();
    inputs.goal = goalState.position;
    inputs.position = motor.getAngularPositionRotations(); // real people hate radians
    inputs.voltage = motor.getInputVoltage();
  }
}
