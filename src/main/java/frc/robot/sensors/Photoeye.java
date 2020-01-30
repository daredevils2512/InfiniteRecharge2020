package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class Photoeye {
  private final DigitalInput m_photoeye;

  public Photoeye(final int channel) {
    m_photoeye = new DigitalInput(channel);
  }

  public boolean get() {
    return m_photoeye.get();
  }
}