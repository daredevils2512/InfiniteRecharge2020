package frc.robot.utils;

import java.util.logging.Logger;

public class MagazinePowerCellCounter {
  private final Logger m_logger;

  private int m_magazineCount;

  public MagazinePowerCellCounter() {
    m_logger = Logger.getLogger(MagazinePowerCellCounter.class.getName());
  }

  public void incrementCount() {
    m_magazineCount++;
    if (m_magazineCount > 3) {
      m_logger.warning("Magazine power cell count exceeded max (3)!");
    }
    m_magazineCount = Math.min(m_magazineCount, 3);
  }

  public void decrementCount() {
    m_magazineCount--;
    if (m_magazineCount < 0) {
      m_logger.warning("Magazine power cell count less than zero!");
    }
    m_magazineCount = Math.max(m_magazineCount, 0);
  }

  public int getCount() {
    return m_magazineCount;
  }
}
