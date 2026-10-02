package frc.robot.subsystems.Elevator;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Robot;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.controls.Follower;

public class ElevatorIOTalonFX implements ElevatorIO {
private static final String ElevatorConfig = null;
private static final String rampCANBus = null;
private static final String PhysicsSim = null; 
public final TalonFX primaryElevatorMotor;
public final TalonFX secondaryElevatorMotor;
public final CANrange canRangeElevator; 


 @SuppressWarnings("unused")
private final MotionMagicTorqueCurrentFOC magicRequest = new MotionMagicTorqueCurrentFOC(0);


 void publicElevatoIOTalonFX() {
  primaryElevatorMotor = new TalonFX(ElevatorConfig.primaryElevatorMotorID, ElevatorConfig);
secondaryElevatorMotor = new TalonFX(ElevatorConfig.secondaryElevatorMotorID, ElevatorConfig);
canRangeElevator = new CANrange(ElevatorConfig.canRangeID, rampCANBus);
if (Robot.isSimulation()) {
   PhysicsSim.getInstance().addTallonFX(primaryElevatorMotor);
}
}
}

