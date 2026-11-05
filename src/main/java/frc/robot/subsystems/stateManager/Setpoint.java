package frc.robot.subsystems.stateManager;

public enum Setpoint {
  /* 3 sets of 3 nodes (9 total):
   *
   * x   o   x | x   o   x | x   o   x
   * x   o   x | x   o   x | x   o   x
   * ?   ?   ? | ?   ?   ? | ?   ?   ?
   *
   * 1   2   3   4   5   6   7   8   9
   */

  // TODO: actually find these values
  CONE_HIGH(3.5, 3.5),
  CONE_MID(2.5, 2.5),
  CONE_LOW(1.5, 1.5),

  CUBE_HIGH(3.0, 3.0),
  CUBE_MID(2.0, 2.0),
  CUBE_LOW(1.0, 1.0),

  INTAKING(2.5, 1.5),

  HOLDING(0.25, 0.5),
  STOW(0, 0.5),
  DYNAMIC(0, 0);

  public double pivotPos;
  public double elevatorPos;

  Setpoint(double pivotPos, double elevatorPos) {
    this.pivotPos = pivotPos;
    this.elevatorPos = elevatorPos;
  }
}
