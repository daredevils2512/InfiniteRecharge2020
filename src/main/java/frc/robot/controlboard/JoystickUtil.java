package frc.robot.controlboard;

import frc.robot.utils.DareMathUtil;

public class JoystickUtil {
  private JoystickUtil() {

  }

  public static double deadband(double value, double deadbandValue) {
    double sign = Math.signum(value);
    double unsignedValue = Math.abs(value);
    return sign * DareMathUtil.mapRange(unsignedValue, 0, 1, deadbandValue, 1);
  }
}