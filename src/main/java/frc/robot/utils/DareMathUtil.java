package frc.robot.utils;

public final class DareMathUtil {
  private DareMathUtil() {
  }

  public static double mapRange(double value, double oldMin, double oldMax, double newMin, double newMax) {
    double valueNormalized = (value - oldMin) / (oldMax - oldMin);
    return valueNormalized * (newMax - newMin) + newMin;
  }

  public static double wrap(double value, double min, double max) {
    if (min > max) {
      double temp = min;
      min = max;
      max = temp;
    }

    double sign = Math.signum(value);
    double range = max - min;
    return (value - sign * min) % range + sign * min;
  }

  /**
   * checks if value is within tolerance x of target
   * @param value input value
   * @param target target that value might be near
   * @param tolerance acceptable error
   * @return true if value is within acceptable error of the target
   */
  public static boolean isWithinXOf(double value, double target, double tolerance) {
    return Math.abs(value - target) <= tolerance;
  }
}