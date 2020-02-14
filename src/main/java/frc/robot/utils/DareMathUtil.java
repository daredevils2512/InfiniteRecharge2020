package frc.robot.utils;

import edu.wpi.first.wpiutil.math.MathUtil;

public final class DareMathUtil {
  private DareMathUtil() {

  }

  public static double clamp(double value, double low, double high) {
    return MathUtil.clamp(value, low, high);
  }

  public static int clamp(int value, int low, int high) {
    return MathUtil.clamp(value, low, high);
  }

  public static double mapRange(double value, double oldMin, double oldMax, double newMin, double newMax) {
    double valueNormalized = (value - oldMin) / (oldMax - oldMin);
    return valueNormalized * (newMax - newMin) + newMin;
  }
}