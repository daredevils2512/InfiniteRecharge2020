package frc.robot.utils;

import edu.wpi.first.wpiutil.math.MathUtil;

public class BoundedCounter extends Counter {
  protected final int m_lowerBound;
  protected final int m_upperBound;

  public BoundedCounter(int lowerBound, int upperBound) {
    m_lowerBound = lowerBound;
    m_upperBound = upperBound;
    m_count = lowerBound;
  }

  public BoundedCounter(int lowerBound, int upperBound, int count) {
    m_lowerBound = lowerBound;
    m_upperBound = upperBound;
    m_count = MathUtil.clamp(count, lowerBound, upperBound);
  }

  @Override
  public void increment() {
    if (m_count < m_upperBound) {
      super.increment();
    }
  }

  @Override
  public void decrement() {
    if (m_count > m_lowerBound) {
      super.decrement();
    }
  }

  @Override
  public void changeCount(int amount) {
    int newCount = MathUtil.clamp(m_count + amount, m_lowerBound, m_upperBound);
    super.setCount(newCount);
  }

  @Override
  public void setCount(int count) {
    count = MathUtil.clamp(count, m_lowerBound, m_upperBound);
    super.setCount(count);
  }
}
