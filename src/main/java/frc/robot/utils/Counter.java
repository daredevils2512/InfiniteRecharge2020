package frc.robot.utils;

public class Counter {
  protected int m_count;

  public Counter() {
    m_count = 0;
  }

  public Counter(int count) {
    m_count = count;
  }

  public void increment() {
    changeCount(1);
  }

  public void decrement() {
    changeCount(-1);
  }

  public void changeCount(int amount) {
    m_count += amount;
  }

  public void setCount(int count) {
    m_count = count;
  }

  public int getCount() {
    return m_count;
  }
}
