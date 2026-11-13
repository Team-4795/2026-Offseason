package frc.robot.subsystems.effector;

public class EffectorConstants {
  public static final int CAN_ID = 11;

  public static final int FREE_CURRENT_LIMIT = 60;
  public static final int STALL_CURRENT_LIMIT = 30;

  public static final boolean VELOCITY_MODE = true; // false: current mode, for gamepiece sensing
  public static final double DEBOUNCE_TIME = 0.5;

  public static final double VELOCITY_THRESHOLD = 3.0;
  public static final double CURRENT_THRESHOLD = 40;

  public static class SensorConstants {
    public static final int SENSOR_ID = 10;
    public static final double FOV_X = 7.0;
    public static final double FOV_Y = 7.0;
    public static final double PROXIMITY_THRESHOLD = 0.13;
  }
}
