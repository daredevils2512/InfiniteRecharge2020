package frc.robot.commands;

import java.util.function.IntSupplier;
import java.util.function.Supplier;

import frc.robot.subsystems.interfaces.IQueue;
import frc.robot.subsystems.interfaces.IShooter;
import frc.robot.utils.DareMathUtil;
import frc.robot.utils.MagazinePowerCellCounter;

public class AutoFeedShooter extends RunQueueCommand {

  private final IShooter m_shooter;
  private double m_targetVelocity;
  private final double m_maxVelocityError;
  private final Supplier<Double> m_targetVelocitySupplier;


  /**
   * auto feed shooter
   * @param queue queue object to run
   * @param queueSpeed set speed for the queue to run at
   * @param shooter shooter object to get speed
   * @param targetVelocitySupplier supplier for target velocity
   * @param maxVelocityError a tolerance of sorts
   */
  public AutoFeedShooter(IQueue queue, double queueSpeed, IShooter shooter, Supplier<Double> targetVelocitySupplier,
      double maxVelocityError) {
    super(queue, queueSpeed);
    m_shooter = shooter;
    m_targetVelocitySupplier = targetVelocitySupplier;
    m_maxVelocityError = maxVelocityError;
  }

  @Override
  protected boolean shouldRunQueue() {
    boolean shouldRunQueue = m_queue.hasPowerCell()
        && DareMathUtil.isWithinXOf(m_shooter.getVelocity(), m_targetVelocity, m_maxVelocityError);
    return shouldRunQueue;
  }
}