package frc.robot.utils;

public class LowerBoundedCounter extends Counter {
  private final int m_lowerBound;

  public LowerBoundedCounter(int lowerBound) {
    m_lowerBound = lowerBound;
    m_count = lowerBound;
  }

  public LowerBoundedCounter(int lowerBound, int count) {
    m_lowerBound = lowerBound;
    m_count = Math.max(lowerBound, count);
  }

  @Override
  public void decrement() {
    if (m_count > m_lowerBound) {
      super.decrement();
    }
  }

  @Override
  public void changeCount(int amount) {
    int newCount = Math.max(m_lowerBound, m_count + amount);
    super.setCount(newCount);
  }

  @Override
  public void setCount(int count) {
    count = Math.max(m_lowerBound, count);
    super.setCount(count);
  }
}
