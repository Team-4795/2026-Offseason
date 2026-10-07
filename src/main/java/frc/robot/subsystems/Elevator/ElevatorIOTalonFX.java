package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;

public class ElevatorIOTalonFX implements ElevatorIO {

  private static final String ElevatorConfig = null;
  private static final String rampCANBus = null;
  private static final String PhysicsSim = null;
  public static final int secondaryElevatorMotorId = 0;
  public static final int secondaryelevatorMotorId = 0;
  public static final int canRangeID = 0;

  @SuppressWarnings("unused")
  private final MotionMagicTorqueCurrentFOC magicRequest = new MotionMagicTorqueCurrentFOC(0);

  public void publicElevatorIOTalonFX() {
    TalonFX primaryElevatorMotor =
        new TalonFX(ElevatorConfig.primaryElevatorMotorId, ElevatorConfig, rampCANBus);
    TalonFX secondaryElevatorMotor =
        new TalonFX(ElevatorConfig.SecondaryElevatorMotorId, ElevatorConfig, rampCANBus);
    CANrange canRangeElevator = new CANrange(ElevatorConfig.canRangeId, rampCANBus);
    if (Robot.isSimulation()) {
      PhysicsSim.getInstance().addTallonFX(primaryElevatorMotor);
    }
  }
}
