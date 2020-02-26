package frc.robot.utils;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.sensors.IDigitalInput;
import frc.robot.subsystems.interfaces.IMagazine;


public class MagazinePowerCellCounter {
  private final Logger m_logger;

  private int m_magazineCount = 0;
  
  private double  m_previousTime; //state buffer
  private double m_errorTime = 0.1; //longest time in between 'balls' going in or out that wed want to ignore

  private boolean m_previousMagazineEye = false;
  private boolean m_previousQueueEye = false;

  private IDigitalInput m_magazineEye;
  private IDigitalInput m_queueEye;
  private IMagazine m_magazine;

  public MagazinePowerCellCounter(IDigitalInput magazineEye, IDigitalInput queueEye, IMagazine magazine) {
    m_logger = Logger.getGlobal();
    m_magazineEye = magazineEye;
    m_queueEye = queueEye;
    m_magazine = magazine;
  }

  public void addToCount(int change) {
    double time = Timer.getFPGATimestamp();
    if (!(time - m_previousTime <= m_errorTime)) {
      m_magazineCount = m_magazineCount + change;
      if (m_magazineCount < 0) {
        m_logger.warning("Magazine power cell count less than zero!");
      }
      m_magazineCount = MathUtil.clamp(m_magazineCount, 0, 4);
    } else {
      m_logger.warning("ball move too quickly : " + (time - m_previousTime));
    }
    m_previousTime = time;
  }

  /**
   * this holds the counting logic -- may need to change ima call it a work in progress
   */
  public void updateCount() {
    if (m_magazineEye.get() && !m_previousMagazineEye) {
      m_previousMagazineEye = true;
      if (m_magazine.getDirectionReversed()) {
        m_logger.fine("ball reversed");
        addToCount(-1);;
      } else {
        m_logger.fine("ball in");
        addToCount(1);;
      }
    }

    if (!m_queueEye.get() && m_previousQueueEye) {
      m_logger.fine("ball out");
      addToCount(-1);
      m_previousQueueEye = false;
    }

    m_previousMagazineEye = m_magazineEye.get();
    m_previousQueueEye = m_queueEye.get();
  }

  public int getCount() {
    SmartDashboard.putNumber("power cell count", m_magazineCount);
    return m_magazineCount;
  }
}
