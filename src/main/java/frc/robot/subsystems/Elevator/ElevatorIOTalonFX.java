package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class ElevatorIOTalonFX implements ElevatorIO {
  // private final PositionTorqueCurrentFOC m_request = new PositionTorqueCurrentFOC(0).withSlot(0);
  private final MotionMagicTorqueCurrentFOC m_request =
      new MotionMagicTorqueCurrentFOC(0); // its just fancy profiled pid

  private final MotionMagicConfigs mmConfigs = new MotionMagicConfigs();

  private double goal = 0.0;

  private final TalonFX rightMotor = new TalonFX(ElevatorConstants.CAN_ID_RIGHT);
  private final TalonFX leftMotor = new TalonFX(ElevatorConstants.CAN_ID_LEFT);

  public ElevatorIOTalonFX() {
    // BaseStatusSignal.setUpdateFrequencyForAll(50, null);

    var config = new TalonFXConfiguration();
    config.Audio.BeepOnBoot = true;
    config.Audio.BeepOnConfig = true;

    config.Feedback.SensorToMechanismRatio = ElevatorConstants.GEARING;

    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.MAX_HEIGHT;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.MIN_HEIGHT;

    config.Slot0.kP = ElevatorConstants.kP;
    config.Slot0.kI = ElevatorConstants.kI;
    config.Slot0.kD = ElevatorConstants.kD;

    config.Slot0.kS = ElevatorConstants.kS;
    config.Slot0.kA = ElevatorConstants.kA;
    config.Slot0.kV = ElevatorConstants.kV;
    config.Slot0.kG = ElevatorConstants.kG;

    mmConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MM_V;
    mmConfigs.MotionMagicAcceleration = ElevatorConstants.MM_A;

    rightMotor.getConfigurator().apply(config);
    rightMotor.getConfigurator().apply(mmConfigs);

    leftMotor.getConfigurator().apply(config);
    leftMotor.getConfigurator().apply(mmConfigs); // just in case
    leftMotor.setControl(new Follower(ElevatorConstants.CAN_ID_RIGHT, MotorAlignmentValue.Opposed));
  }

  @Override
  public void setGoal(double goal) {
    this.goal = goal;
    rightMotor.setControl(m_request.withPosition(goal));
  }

  // TODO right now it only logs information about one motor, should log the other motor too
  @Override
  public void updateInputs(ElevatorIOInputs input) {
    input.goal = goal;

    input.elevatorRightPositionMeters = rightMotor.getPosition().getValueAsDouble();
    input.elevatorRightStatorCurrent = rightMotor.getSupplyCurrent().getValueAsDouble();
    input.elevatorRightAppliedVolts = rightMotor.getSupplyVoltage().getValueAsDouble();
    input.elevatorRightVelocityMetersPerSecond = rightMotor.getVelocity().getValueAsDouble();
  }
}
