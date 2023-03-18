package frc.util;

public class Slew {
  public static double GetSlewedTarget(double slew, double target, double current) {
    double direction = target - current > 0 ? 1 : -1;
    double delta = Math.min(slew, Math.abs(target - current)) * direction;
    return current + delta;
  }
}
