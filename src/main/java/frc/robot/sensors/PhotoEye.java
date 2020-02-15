package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class PhotoEye {
  private final DigitalInput m_photoEye;

  public PhotoEye(int channel) {
    m_photoEye = new DigitalInput(channel);
  }

  public boolean get() {
    return !m_photoEye.get();
  }
}