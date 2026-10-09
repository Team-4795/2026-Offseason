package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIOTalonFX implements ElevatorIO {

  private static final String rampCANBus = null;
  private static final String PhysicsSim = null;

  @SuppressWarnings("unused")
  private final MotionMagicTorqueCurrentFOC magicRequest = new MotionMagicTorqueCurrentFOC(0);

  private final TalonFX primaryElevatorMotor =
      new TalonFX(ElevatorConstants.primaryElevatorMotorId, rampCANBus);
  private final TalonFX secondaryElevatorMotor =
      new TalonFX(ElevatorConstants.secondaryElevatorMotorId, rampCANBus);

  public ElevatorIOTalonFX() {
    final TalonFX primaryElevatorMotor =
        new TalonFX(ElevatorConstants.primaryElevatorMotorId, rampCANBus);
    final TalonFX secondaryElevatorMotor =
        new TalonFX(ElevatorConstants.secondaryElevatorMotorId, rampCANBus);
  }
}
