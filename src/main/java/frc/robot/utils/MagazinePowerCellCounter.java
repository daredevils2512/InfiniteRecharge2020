package frc.robot.utils;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.IDigitalInput;
import frc.robot.sensors.PhotoEye;
import frc.robot.subsystems.interfaces.IMagazine;


public class MagazinePowerCellCounter {
  private final Logger m_logger;

  private int m_magazineCount = 0;
  private double  m_previousTimeAdd = 0.0; //buffer for addition
  private double m_previousTimeSub = 0.0; //buffer for subtraction
  private double m_errorTime = 0.1; //longest time in between 'balls' going in or out that wed want to ignore

  private boolean m_previousMagazineEye = false;
  private boolean m_previousQueueEye = false;

  private IDigitalInput m_magazineEye;
  private IDigitalInput m_queueEye;
  private IMagazine m_magazine;

  public MagazinePowerCellCounter(IDigitalInput magazineEye, IDigitalInput queueEye, IMagazine magazine) {
    m_logger = Logger.getLogger(MagazinePowerCellCounter.class.getName());
    m_magazineEye = magazineEye;
    m_queueEye = queueEye;
    m_magazine = magazine;
  }

  public void incrementCount() {
    double time = Timer.getFPGATimestamp();
    if (!(time - m_previousTimeAdd <= m_errorTime)) {
      m_magazineCount++;
      if (m_magazineCount > 3) {
        m_logger.warning("Magazine power cell count exceeded max (3)!");
      }
      m_magazineCount = Math.min(m_magazineCount, 3);
    } else {
      System.out.println("ball move too quickly : " + (time - m_previousTimeAdd));
    }
    m_previousTimeAdd = time;
  }

  public void decrementCount() {
    double time = Timer.getFPGATimestamp();
    if (!(time - m_previousTimeSub <= m_errorTime)) {
      m_magazineCount--;
      if (m_magazineCount < 0) {
        m_logger.warning("Magazine power cell count less than zero!");
      }
      m_magazineCount = Math.max(m_magazineCount, 0);
    } else {
      System.out.println("ball move too quickly : " + (time - m_previousTimeSub));
    }
    m_previousTimeSub = time;
  }

  public void updateCount() {
    if (m_magazineEye.get() && !m_previousMagazineEye) {
      m_previousMagazineEye = true;
      if (m_magazine.getDirectionReversed()) {
        System.out.println("ball reversed");
        decrementCount();
      } else {
        System.out.println("ball in");
        incrementCount();
      }
    }

    if (!m_queueEye.get() && m_previousQueueEye) {
      System.out.println("ball out");
      decrementCount();
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
