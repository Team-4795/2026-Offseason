package frc.robot.subsystems.elevator;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        public double voltage = 0;
        public double setpointPosition = 0;
        public double setpointVoltage;
        public double goalPosition = 0;
        public double goalVoltage;
        public double current = 0;
    }

    public default void setVoltage(double v) {}

    public default void setGoal(double v) {}

    public default double getPosition() {
        return 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

}
