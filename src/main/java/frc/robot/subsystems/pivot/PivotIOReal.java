package frc.robot.subsystems.pivot;

import java.util.InputMismatchException;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;

import edu.wpi.first.math.controller.PIDController;

public class PivotIOReal implements PivotIO {

    final TalonFX motor = new TalonFX(PivotConstants.motorID1);
    TalonFXConfiguration configs = new TalonFXConfiguration();

    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    double setGoal = 0.0;

    @Override
    public void Initialize() 
    {
        configs.Slot0.kP = PivotConstants.P;
        configs.Slot0.kI = PivotConstants.I;
        configs.Slot0.kD = PivotConstants.D;
        configs.Slot0.kA = PivotConstants.A;
        configs.Slot0.kG = PivotConstants.G;
        configs.Slot0.kS = PivotConstants.S;
        configs.Slot0.kV = PivotConstants.V;
        motor.getConfigurator().apply(configs);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }
    @Override
    public void setGoal(double goal)
    {
        setGoal = goal;
        motor.setControl(m_request.withPosition(goal));
    }
    @Override
    public void updateInputs(PivotIOInputs inputs) 
    {
        inputs.velocity = motor.getVelocity().getValueAsDouble();
        inputs.current = motor.getSupplyCurrent().getValueAsDouble();
        inputs.goal = setGoal;
        inputs.position = motor.getPosition().getValueAsDouble();
        inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
    }
}
