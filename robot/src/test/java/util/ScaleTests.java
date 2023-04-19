package util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.util.Scale;

public class ScaleTests {
  @Test
  void PercentageAtLow() {
    double percent = 0;
    double low = .2;
    double high = 1;
    double result = Scale.FromPercentage(percent, low, high);
    assertEquals(.2, result);
  }

  @Test
  void PercentageQuarterWay() {
    double percent = .25;
    double low = .2;
    double high = 1;
    double result = Scale.FromPercentage(percent, low, high);
    assertEquals(.4, result);
  }

  @Test
  void PercentageAtHigh() {
    double percent = 1;
    double low = .2;
    double high = 1;
    double result = Scale.FromPercentage(percent, low, high);
    assertEquals(1, result);
  }
}
