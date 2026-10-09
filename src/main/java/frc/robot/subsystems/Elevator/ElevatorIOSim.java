package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorIOInputs inputs = new ElevatorIOInputs();
  private PIDController leftController = new PIDController(0, 0, 0);
  private ProfiledPIDController rightController = new ProfiledPIDController(0, 0, 0, null);
  private ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);

  private void updateInputs() {
    // Update the inputs with simulated values so that the Elevator subsystems can use them for
    // control and logging
    inputs.elevatorLeftPositionMeters = 0; // Replace dis  simulated position
    inputs.elevatorLeftVelocityMetersPerSecond = 0; // Replace dis  simulated velocity
    inputs.elevatorLeftCurrent = 0; // Replace with  simulated current
    inputs.elevatorRIghtPositionMeters = 0; // Replace with  simulated position
    inputs.elevatorRightVelocityMetersPerSecond = 0; // Replace with  simulated velocity
    inputs.elevatorRightCurrent = 0; // Replace with  simulated current
  }

  private static final double MIN_HEIGHT = 0.0;
  private static final double MAX_HEIGHT = 0.0;
  private final ProfiledPIDController feedback =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
}
