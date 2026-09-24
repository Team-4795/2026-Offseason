package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class PivotIOReal implements PivotIO {
  /* general comments:
   * careful with your capitalization and naming
   *
   * add:
   * logged tunables
   * configure method (for tuning)
   */

  private final TalonFX motor = new TalonFX(PivotConstants.CAN_ID_1);
  private TalonFXConfiguration configs = new TalonFXConfiguration();

  private final PositionTorqueCurrentFOC m_request =
      new PositionTorqueCurrentFOC(0).withSlot(0); // we have pro so we can do this
  private double goal = 0.0;

  private final StatusSignal<Angle> position = motor.getPosition(); // rotations
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity(); // rps
  private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getTorqueCurrent();

  public PivotIOReal() { // just make these normal constructors
    // these are not that important but good to have
    configs.Audio.BeepOnBoot = true;
    configs.Audio.BeepOnConfig = true;

    // THESE ARE VERY IMPORTANT
    configs.CurrentLimits.StatorCurrentLimit = PivotConstants.STATOR_CURRENT_LIMIT;
    configs.CurrentLimits.SupplyCurrentLimit = PivotConstants.SUPPLY_CURRENT_LIMIT;
    configs.Feedback.SensorToMechanismRatio = PivotConstants.GEARING;

    configs.Slot0.kP = PivotConstants.kP;
    configs.Slot0.kI = PivotConstants.kI;
    configs.Slot0.kD = PivotConstants.kD;
    configs.Slot0.kA = PivotConstants.kA;
    configs.Slot0.kG = PivotConstants.kG;
    configs.Slot0.kS = PivotConstants.kS;
    configs.Slot0.kV = PivotConstants.kV;
    motor.getConfigurator().apply(configs);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void setGoal(double goal) {
    this.goal = goal;
    motor.setControl(m_request.withPosition(goal));
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, voltage, current);
    inputs.velocity = velocity.getValueAsDouble();
    inputs.current = current.getValueAsDouble();
    inputs.goal = goal;
    inputs.position = position.getValueAsDouble();
    inputs.voltage = voltage.getValueAsDouble();
  }
}
