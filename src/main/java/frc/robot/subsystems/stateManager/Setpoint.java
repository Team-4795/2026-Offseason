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
  CONE_HIGH(0, 0),
  CONE_MID(0, 0),
  CONE_LOW(0, 0),

  CUBE_HIGH(0, 0),
  CUBE_MID(0, 0),
  CUBE_LOW(0, 0),

  HOLDING(0, 0),
  STOW(0, 0);

  public double pivotPos;
  public double elevatorPos;

  Setpoint(double pivotPos, double elevatorPos) {
    this.pivotPos = pivotPos;
    this.elevatorPos = elevatorPos;
  }
}
