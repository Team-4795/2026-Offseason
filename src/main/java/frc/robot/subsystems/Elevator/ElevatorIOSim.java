package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {

  private DCMotorSim simMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(2), 0.1, 2),
          DCMotor.getKrakenX60(2));
  private TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration);
  private TrapezoidProfile profile = new TrapezoidProfile(constraints);
  private ProfiledPIDController pidController =
      new ProfiledPIDController(
          ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, constraints);
  private SimpleMotorFeedforward ffmodel =
      new SimpleMotorFeedforward(ElevatorConstants.kS, ElevatorConstants.kV, ElevatorConstants.kA);

  private TrapezoidProfile.State goalState = new TrapezoidProfile.State(0.0, 0.0);
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State(0.0, 0.0);

  double goalSet = 0.0;

  @Override
  public void setGoal(double goal) {
    double clampedGoal = goal;
    if (goalState.position != goal) {
      clampedGoal = MathUtil.clamp(goal, ElevatorConstants.position1, ElevatorConstants.position2);
      goalState = new TrapezoidProfile.State(clampedGoal, 0.0);
      setpointState =
          new TrapezoidProfile.State(
              simMotor.getAngularPositionRad(), simMotor.getAngularVelocityRadPerSec());
    }
    setpointState = profile.calculate(0.02, setpointState, goalState);

    simMotor.setInputVoltage(
        ffmodel.calculate(setpointState.velocity)
            + pidController.calculate(
                simMotor.getAngularPositionRotations(), setpointState.position));
    goalSet = goal;
  }

  @Override
  public double getPosition() {
    return simMotor.getAngularPositionRad();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.current = simMotor.getCurrentDrawAmps();
    inputs.voltage = simMotor.getInputVoltage();
    inputs.setpointPosition = goalSet;
  }
}
