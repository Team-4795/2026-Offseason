package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {
  private ProfiledPIDController feedback =
      new ProfiledPIDController(0.5, 0.0, 1.5, new TrapezoidProfile.Constraints(50, 10));
  private ElevatorFeedforward feedforward = new ElevatorFeedforward(0.0, 0.0, 0);

  private final DCMotorSim motor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(2), 0.08, ElevatorConstants.SIM_GEARING),
          DCMotor.getKrakenX60(1));
  private double voltage = 0.0;
  private double goal = 0.0;

  @Override
  public void setGoal(double goal) {
    this.goal = goal;
  }

  @Override
  public double getGoal() {
    return goal;
  }

  @Override
  public void updateInputs(ElevatorIOInputs input) {
    voltage =
        feedback.calculate(motor.getAngularPositionRotations(), goal)
            + feedforward.calculate(motor.getAngularVelocityRadPerSec());
    motor.setInputVoltage(voltage);

    motor.update(0.02);

    input.rightPositionMeters = motor.getAngularPositionRotations();
    input.rightStatorCurrent = motor.getCurrentDrawAmps();
    input.rightAppliedVolts = voltage;
    input.rightVelocityMetersPerSecond = motor.getAngularVelocityRPM();
    input.setpointVel = feedback.getSetpoint().velocity;
    input.setpointPos = feedback.getSetpoint().velocity;
    input.goal = goal;
  }
}
