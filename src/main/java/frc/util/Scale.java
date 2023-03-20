package frc.util;

public class Scale {
  /**
   * From a percentage, give a value between the two given numbers.
   * @param percentage A number between 0 and 1.
   * @param low The number to return if percentage is 0.
   * @param high The number to return if percentage is 1.
   * @return
   */
  public static double FromPercentage(double percentage, double low, double high) {
    double diff = high - low;
    double delta = diff * percentage;
    return low + delta;
  }
}
