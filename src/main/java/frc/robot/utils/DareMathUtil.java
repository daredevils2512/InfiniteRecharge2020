package frc.robot.utils;

import edu.wpi.first.wpiutil.math.MathUtil;

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
}