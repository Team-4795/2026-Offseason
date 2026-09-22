package frc.robot.subsystems.drive.effector;

import org.littletonrobotics.junction.AutoLog;

public interface EffectorSensorIO {
    
    public default double getDistance() {return 6700.0;};
}
