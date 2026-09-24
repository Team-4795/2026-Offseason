package frc.robot.subsystems.effector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class EffectorIOSim implements EffectorIO {
  private DCMotorSim effectorMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 2),
          DCMotor.getKrakenX60(1));
  private double currentVoltage = 0;
}
