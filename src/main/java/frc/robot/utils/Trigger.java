package frc.robot.utils;

import java.util.function.BooleanSupplier;

public class Trigger {
  private final BooleanSupplier m_isActive;

  private Runnable m_whenActive = () -> { };
  private Runnable m_whileActive = () -> { };
  private Runnable m_whenInactive = () -> { };
  private Runnable m_whileInactive = () -> { };

  private boolean m_previouslyActive;

  public Trigger() {
    m_isActive = () -> false;
  }

  public Trigger(BooleanSupplier isActive) {
    m_isActive = isActive;
  }

  public boolean get() {
    return m_isActive.getAsBoolean();
  }

  /**
   * Run once when trigger becomes active
   * @param runnable Runnable to run
   */
  public void whenActive(Runnable runnable) {
    m_whenActive = runnable;
  }

  /**
   * Run continuously while trigger is active
   * @param runnable Runnable to run
   */
  public void whileActive(Runnable runnable) {
    m_whileActive = runnable;
  }

  /**
   * Run once when trigger becomes inactive
   * @param runnable Runnable to run
   */
  public void whenInactive(Runnable runnable) {
    m_whenInactive = runnable;
  }

  /**
   * Run continuously while trigger is inactive
   * @param runnable Runnable to run
   */
  public void whileInactive(Runnable runnable) {
    m_whileInactive = runnable;
  }

  /**
   * Update the trigger, running any applicable runnables
   */
  public void update() {
    boolean isActive = get();

    if (isActive && !m_previouslyActive) {
      m_whenActive.run();
    } else if (isActive && m_previouslyActive) {
      m_whileActive.run();
    } else if (!isActive && m_previouslyActive) {
      m_whenInactive.run();
    } else if (!isActive && !m_previouslyActive) {
      m_whileInactive.run();
    }

    m_previouslyActive = isActive;
  }
}
