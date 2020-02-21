package frc.robot.utils;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj.Timer;


public class MagazinePowerCellCounter {
  private final Logger m_logger;

  private int m_magazineCount = 0;
  private double  m_previousTimeAdd = 0.0; //buffer for addition
  private double m_previousTimeSub = 0.0; //buffer for subtraction
  private double m_errorTime = 0.1; //longest time in between 'balls' going in or out that wed want to ignore

  public MagazinePowerCellCounter() {
    m_logger = Logger.getLogger(MagazinePowerCellCounter.class.getName());
  }

  public void incrementCount() {
    double time = Timer.getFPGATimestamp();
    if (time - m_previousTimeAdd <= m_errorTime) {
      m_magazineCount++;
      if (m_magazineCount > 3) {
        m_logger.warning("Magazine power cell count exceeded max (3)!");
      }
      m_magazineCount = Math.min(m_magazineCount, 3);
    }
    m_previousTimeAdd = time;
  }

  public void decrementCount() {
    double time = Timer.getFPGATimestamp();
    if (time - m_previousTimeSub <= m_errorTime) {
      m_magazineCount--;
      if (m_magazineCount < 0) {
        m_logger.warning("Magazine power cell count less than zero!");
      }
      m_magazineCount = Math.max(m_magazineCount, 0);
    }
    m_previousTimeSub = time;
  }

  public int getCount() {
    return m_magazineCount;
  }
}
