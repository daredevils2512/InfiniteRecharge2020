package frc.robot.utils;

import edu.wpi.first.wpiutil.math.MathUtil;

public class BoundedCounter extends Counter {
  protected final int m_lowerBound;
  protected final int m_upperBound;

  public BoundedCounter(int lowerBound, int upperBound) {
    m_lowerBound = lowerBound;
    m_upperBound = upperBound;
  }

  public BoundedCounter(int lowerBound, int upperBound, int count) {
    super(MathUtil.clamp(count, lowerBound, upperBound));
    m_lowerBound = lowerBound;
    m_upperBound = upperBound;
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
    // TODO: Bound
    super.changeCount(amount);
  }
}
