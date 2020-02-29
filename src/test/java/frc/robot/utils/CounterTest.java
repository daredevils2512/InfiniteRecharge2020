package frc.robot.utils;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import org.junit.Test;

public class CounterTest {
  @Test
  public void testCounter() {
    int startCount = 1;
    int changeCount = 2;
    int setCount = 3;
    int lastCount;

    Counter counter = new Counter(startCount);
    assertEquals(startCount, counter.getCount());
    lastCount = counter.getCount();

    counter.increment();
    assertEquals(lastCount + 1, counter.getCount());
    lastCount = counter.getCount();

    counter.decrement();
    assertEquals(lastCount - 1, counter.getCount());
    lastCount = counter.getCount();

    counter.changeCount(changeCount);
    assertEquals(lastCount + changeCount, counter.getCount());
    lastCount = counter.getCount();

    counter.setCount(setCount);
    assertEquals(setCount, counter.getCount());
  }

  @Test
  public void testLowerBoundedCounter() {
    int lowerBound = 1;
    int startCount = 0;

    LowerBoundedCounter counter = new LowerBoundedCounter(lowerBound, startCount);
    assertFalse(counter.getCount() < lowerBound);

    counter.setCount(lowerBound);
    counter.decrement();
    assertFalse(counter.getCount() < lowerBound);
    
    counter.setCount(lowerBound);
    counter.changeCount(-1);
    assertFalse(counter.getCount() < lowerBound);

    counter.setCount(lowerBound - 1);
    assertFalse(counter.getCount() < lowerBound);
  }
}
