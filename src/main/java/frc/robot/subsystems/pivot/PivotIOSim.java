package frc.robot.subsystems.pivot;
import java.util.InputMismatchException;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
public class PivotIOSim implements PivotIO {

    private DCMotorSim motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1),0.0001,2),DCMotor.getKrakenX60(1));
    TalonFXConfiguration configs = new TalonFXConfiguration();
    private PIDController Pid = new PIDController(PivotConstants.P, PivotConstants.I, PivotConstants.D);
    private SimpleMotorFeedforward feedFoward = new SimpleMotorFeedforward(PivotConstants.S,PivotConstants.V,PivotConstants.A);
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    double setGoal = 0.0;

    @Override
    public void Initialize() 
    {
        configs.Slot0.kA = PivotConstants.A;
        configs.Slot0.kG = PivotConstants.G;
        configs.Slot0.kS = PivotConstants.S;
        configs.Slot0.kV = PivotConstants.V;
        

    }

    @Override
    public void setVoltage(double voltage) {
        motor.setInputVoltage(voltage);
    }
    @Override
    public void setGoal(double goal)
    {
        setGoal = goal;
        motor.setInputVoltage(Pid.calculate(motor.getAngularPositionRad(),goal) + feedFoward.calculate(goal));
    }
    @Override
    public void updateInputs(PivotIOInputs inputs) 
    {
        inputs.velocity = motor.getAngularVelocityRPM();
        inputs.current = motor.getCurrentDrawAmps();
        inputs.goal = setGoal;
        inputs.position = motor.getAngularPositionRad();
        inputs.voltage = motor.getInputVoltage();
    }
}
