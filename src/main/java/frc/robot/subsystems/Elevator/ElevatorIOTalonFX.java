package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final PositionTorqueCurrentFOC m_request = new PositionTorqueCurrentFOC(0).withSlot(0);

  private final TalonFX primaryElevatorMotor =
      new TalonFX(ElevatorConstants.primaryElevatorMotorId);
  private final TalonFX secondaryElevatorMotor =
      new TalonFX(ElevatorConstants.secondaryElevatorMotorId);

  public ElevatorIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.Slot0.kP = ElevatorConstants.kP;
    config.Slot0.kI = ElevatorConstants.kI;
    config.Slot0.kD = ElevatorConstants.kD;

    config.Slot0.kS = ElevatorConstants.kS;
    config.Slot0.kA = ElevatorConstants.kA;
    config.Slot0.kV = ElevatorConstants.kV;

    primaryElevatorMotor.getConfigurator().apply(config);
    secondaryElevatorMotor.setControl(
        new Follower(ElevatorConstants.primaryElevatorMotorId, MotorAlignmentValue.Opposed));
  }

  @Override
  public void setGoal(double goal) {
    primaryElevatorMotor.setControl(m_request.withPosition(goal));
  }

  @Override
  public void updateInputs(ElevatorIOInputs input) {
    input.elevatorRIghtPositionMeters = primaryElevatorMotor.getPosition().getValueAsDouble();
    input.elevatorRightCurrent = primaryElevatorMotor.getSupplyCurrent().getValueAsDouble();
    input.ElevatorRightAppliedVolts = primaryElevatorMotor.getSupplyVoltage().getValueAsDouble();
    input.elevatorRightVelocityMetersPerSecond =
        primaryElevatorMotor.getVelocity().getValueAsDouble();
  }
}
