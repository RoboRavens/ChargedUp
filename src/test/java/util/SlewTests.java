package util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.util.Slew;

public class SlewTests {
  @Test
  void TargetGreaterThanCurrent_DoesNotReachTarget() {
    double slew = .1;
    double target = 1;
    double current = 0;
    double result = Slew.GetSlewedTarget(slew, target, current);
    assertEquals(.1, result);
  }

  @Test
  void TargetSmallerThanCurrent_DoesNotReachTarget() {
    double slew = .1;
    double target = 0;
    double current = 1;
    double result = Slew.GetSlewedTarget(slew, target, current);
    assertEquals(.9, result);
  }

  @Test
  void NegativeTargetGreaterThanCurrent_DoesNotReachTarget() {
    double slew = .1;
    double target = -1;
    double current = -2;
    double result = Slew.GetSlewedTarget(slew, target, current);
    assertEquals(-1.9, result);
  }

  @Test
  void NegativeTargetSmallerThanCurrent_DoesNotReachTarget() {
    double slew = .1;
    double target = -2;
    double current = -1;
    double result = Slew.GetSlewedTarget(slew, target, current);
    assertEquals(-1.1, result);
  }

  @Test
  void TargetGreaterThanCurrent_DoesReachTarget() {
    double slew = 2;
    double target = 1;
    double current = 0;
    double result = Slew.GetSlewedTarget(slew, target, current);
    assertEquals(target, result);
  }

  @Test
  void TargetSmallerThanCurrent_DoesReachTarget() {
    double slew = 2;
    double target = 0;
    double current = 1;
    double result = Slew.GetSlewedTarget(slew, target, current);
    assertEquals(target, result);
  }

  @Test
  void NegativeTargetGreaterThanCurrent_DoesReachTarget() {
    double slew = 2;
    double target = -1;
    double current = -2;
    double result = Slew.GetSlewedTarget(slew, target, current);
    assertEquals(target, result);
  }

  @Test
  void NegativeTargetSmallerThanCurrent_DoesReachTarget() {
    double slew = 2;
    double target = -2;
    double current = -1;
    double result = Slew.GetSlewedTarget(slew, target, current);
    assertEquals(target, result);
  }
}
