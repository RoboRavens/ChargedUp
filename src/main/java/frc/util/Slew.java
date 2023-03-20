package frc.util;

public class Slew {
  /**
   * Moves the current value towards the target value based on the given slew. If slew is .1, target value is 1, and current value is .5 then the result will be .6.
   * @param slew the rate at which to move current towards target
   * @param target the target value
   * @param current the current value
   * @return The current value moved towards the target value by the amount of the slew.
   */
  public static double GetSlewedTarget(double slew, double target, double current) {
    double direction = target - current > 0 ? 1 : -1;
    double delta = Math.min(slew, Math.abs(target - current)) * direction;
    return current + delta;
  }
}
