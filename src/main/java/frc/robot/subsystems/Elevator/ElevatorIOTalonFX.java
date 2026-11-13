package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {
  // private final PositionTorqueCurrentFOC m_request = new PositionTorqueCurrentFOC(0).withSlot(0);
  private final MotionMagicTorqueCurrentFOC m_request =
      new MotionMagicTorqueCurrentFOC(0); // its just fancy profiled pid

  private final MotionMagicConfigs mmConfigs = new MotionMagicConfigs();

  private double goal = 0.0;

  private final TalonFX rightMotor = new TalonFX(ElevatorConstants.CAN_ID_RIGHT);
  private final TalonFX leftMotor = new TalonFX(ElevatorConstants.CAN_ID_LEFT);

  private StatusSignal<Angle> rightPosition = rightMotor.getPosition();
  private StatusSignal<AngularVelocity> rightVelocity = rightMotor.getVelocity();
  private StatusSignal<Current> rightStatorCurrent = rightMotor.getStatorCurrent();
  private StatusSignal<Voltage> rightVoltage = rightMotor.getMotorVoltage();

  private StatusSignal<Angle> leftPosition = leftMotor.getPosition();
  private StatusSignal<AngularVelocity> leftVelocity = leftMotor.getVelocity();
  private StatusSignal<Current> leftStatorCurrent = leftMotor.getStatorCurrent();
  private StatusSignal<Voltage> leftVoltage = leftMotor.getMotorVoltage();

  private StatusSignal<Double> setpointPos = rightMotor.getClosedLoopReference();
  private StatusSignal<Double> setpointVel = rightMotor.getClosedLoopReferenceSlope();

  public ElevatorIOTalonFX() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        rightPosition,
        rightVelocity,
        rightStatorCurrent,
        rightVoltage,
        leftPosition,
        leftVelocity,
        leftStatorCurrent,
        leftVoltage,
        setpointPos,
        setpointVel);

    PhoenixUtil.addSignals(
        rightPosition,
        rightVelocity,
        rightStatorCurrent,
        rightVoltage,
        leftPosition,
        leftVelocity,
        leftStatorCurrent,
        leftVoltage,
        setpointPos,
        setpointVel);

    var config = new TalonFXConfiguration();
    config.Audio.BeepOnBoot = true;
    config.Audio.BeepOnConfig = true;

    config.Feedback.SensorToMechanismRatio = ElevatorConstants.SIM_GEARING;

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

    PhoenixUtil.tryUntilOk(5, () -> rightMotor.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> rightMotor.getConfigurator().apply(mmConfigs));

    PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(mmConfigs)); // just in case
    leftMotor.setControl(new Follower(ElevatorConstants.CAN_ID_RIGHT, MotorAlignmentValue.Opposed));
  }

  @Override
  public void setGoal(double goal) {
    this.goal = goal;
    rightMotor.setControl(m_request.withPosition(goal));
  }

  @Override
  public void updateInputs(ElevatorIOInputs input) {
    // BaseStatusSignal.refreshAll(rightPosition, rightVelocity, rightStatorCurrent, rightVoltage,
    // leftPosition, leftVelocity, leftStatorCurrent, leftVoltage, setpointPos, setpointVel);

    input.goal = goal;
    input.setpointPos = setpointPos.getValueAsDouble();
    input.setpointVel = setpointVel.getValueAsDouble();

    input.rightPositionMeters = rightPosition.getValueAsDouble();
    input.rightStatorCurrent = rightStatorCurrent.getValueAsDouble();
    input.rightVolts = rightVoltage.getValueAsDouble();
    input.rightVelocityMetersPerSecond = rightVelocity.getValueAsDouble();

    input.leftPositionMeters = leftPosition.getValueAsDouble();
    input.leftStatorCurrent = leftStatorCurrent.getValueAsDouble();
    input.leftVolts = leftVoltage.getValueAsDouble();
    input.leftVelocityMetersPerSecond = leftVelocity.getValueAsDouble();
  }
}
