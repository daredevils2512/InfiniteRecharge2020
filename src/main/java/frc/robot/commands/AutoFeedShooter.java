package frc.robot.commands;

import frc.robot.subsystems.interfaces.IQueue;
import frc.robot.subsystems.interfaces.IShooter;

public class AutoFeedShooter extends RunQueueCommand {

  private final IShooter m_shooter;

  /**
   * auto feed shooter
   * @param queue queue object to run
   * @param queueSpeed set speed for the queue to run at
   * @param shooter shooter object to get speed
   * @param targetVelocitySupplier supplier for target velocity
   * @param maxVelocityError a tolerance of sorts
   */
  public AutoFeedShooter(IQueue queue, double queueSpeed, IShooter shooter) {
    super(queue, queueSpeed);
    m_shooter = shooter;
  }

  @Override
  protected boolean shouldRunQueue() {
    boolean shouldRunQueue = m_queue.hasPowerCell()
        && m_shooter.isAtSpeed();
    return shouldRunQueue;
  }
}