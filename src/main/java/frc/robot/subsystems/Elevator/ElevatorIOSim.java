package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {
  // IMPORTANT IMPORTANT:
  // you won't be able to use CTRE-specific stuff with the version of sim we're currently running
  // so you have to do the old-fashioned pid and ff controllers

  private DCMotorSim motor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), 0.0001, ElevatorConstants.GEARING),
          DCMotor.getKrakenX60(1));
  private TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(ElevatorConstants.MAX_V, ElevatorConstants.MAX_A);
  private TrapezoidProfile profile = new TrapezoidProfile(constraints);
  private ProfiledPIDController pid =
      new ProfiledPIDController(
          ElevatorConstants.SIM_kP,
          ElevatorConstants.SIM_kI,
          ElevatorConstants.SIM_kD,
          constraints);
  private SimpleMotorFeedforward feedFoward =
      new SimpleMotorFeedforward(
          ElevatorConstants.SIM_kS, ElevatorConstants.SIM_kV, ElevatorConstants.SIM_kA);

  private TrapezoidProfile.State goalState = new TrapezoidProfile.State(0.0, 0.0);
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State(0.0, 0.0);

  private double clampedGoal;
  private double pidVolts;
  private double ffVolts;

  public ElevatorIOSim() {}

  @Override
  public void setVoltage(double voltage) {
    motor.setInputVoltage(voltage);
  }

  @Override
  public void setGoal(double goal) {
    if (goalState.position != goal) {
      clampedGoal =
          MathUtil.clamp(goal, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT);
      goalState = new TrapezoidProfile.State(clampedGoal, 0.0);
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
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.voltage = motor.getInputVoltage();
    inputs.setpointPosition = setpointState.position;
    inputs.goalPosition = goalState.position;
    inputs.current = motor.getCurrentDrawAmps();
  }
}
