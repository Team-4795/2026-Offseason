package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {
  private ProfiledPIDController feedback =
      new ProfiledPIDController(0.7, 0.5, 0.7, new TrapezoidProfile.Constraints(50, 10));
  private ElevatorFeedforward feedforward = new ElevatorFeedforward(0.6, 0.3, 0.6);

  private final DCMotorSim motor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.08, 1),
          DCMotor.getKrakenX60(1));
  private double voltage = 0.0;

  @Override
  public void setGoal(double goal) {
    voltage =
        feedback.calculate(motor.getAngularPositionRotations(), goal)
            + feedforward.calculate(motor.getAngularVelocityRadPerSec());
    motor.setInputVoltage(goal);
  }

  @Override
  public void updateInputs(ElevatorIOInputs input) {
    input.elevatorRightPositionMeters = motor.getAngularPositionRotations();
    input.elevatorRightStatorCurrent = motor.getCurrentDrawAmps();
    input.elevatorRightAppliedVolts = voltage;
    input.elevatorRightVelocityMetersPerSecond = motor.getAngularVelocityRPM();
  }
}
