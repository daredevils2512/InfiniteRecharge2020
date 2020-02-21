package frc.robot.sensors;

import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhotoEye implements IDigitalInput {
  private final DigitalInput m_photoEye;

  public PhotoEye(int channel) {
    Logger.getAnonymousLogger().log(Level.INFO, "created a photo eye for DIO channel " + channel);
    m_photoEye = new DigitalInput(channel);
  }

  public boolean get() {
    SmartDashboard.putBoolean("photoeye " + m_photoEye.getChannel(), m_photoEye.get());
    return m_photoEye.get();
  }
}