package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PhotoEye extends Trigger implements IDigitalInput {
  private final DigitalInput m_photoEye;

  public PhotoEye(int channel) {
    m_photoEye = new DigitalInput(channel);
  }

  public boolean get() {
    return !m_photoEye.get();
  }
}