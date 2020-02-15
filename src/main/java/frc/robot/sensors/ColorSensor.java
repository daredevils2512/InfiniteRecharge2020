
package frc.robot.sensors;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  // calibrations used to find colors of the things you are scaning
  private final Color kYellowTarget = ColorMatch.makeColor(0.33, 0.55, 0.11);
  private final Color kRedTarget = ColorMatch.makeColor(0.52, 0.36, 0.14);
  private final Color kGreenTarget = ColorMatch.makeColor(0.18, 0.57, 0.25);
  private final Color kBlueTarget = ColorMatch.makeColor(0.13, 0.43, 0.44);

  public enum ColorDetect {
    Blue, Red, Green, Yellow, CoLoROuToFrAnGe, Unknown
  }

  public ColorSensor() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    m_colorMatcher.setConfidenceThreshold(0.05);
  }

  public Color getColor() {
    return m_colorSensor.getColor();

  }

  public ColorMatchResult getColorMatch() {
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    return match;
  }

  public ColorDetect getColorMatchDetect() {
    ColorMatchResult match = getColorMatch();
    ColorDetect colorDetect;

    if (match.confidence > 0.95) {
      if (match.color == kBlueTarget) {
        colorDetect = ColorDetect.Blue;
      } else if (match.color == kRedTarget) {
        colorDetect = ColorDetect.Red;
      } else if (match.color == kGreenTarget) {
        colorDetect = ColorDetect.Green;
      } else if (match.color == kYellowTarget) {
        colorDetect = ColorDetect.Yellow;
      } else {
        colorDetect = ColorDetect.CoLoROuToFrAnGe;
      }
    } else {
      colorDetect = ColorDetect.Unknown;
    }
    return colorDetect;
  }
}