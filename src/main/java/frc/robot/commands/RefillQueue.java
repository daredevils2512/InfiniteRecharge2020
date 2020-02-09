package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Queue;

public class RefillQueue extends CommandBase {
  private static Queue m_queue;
  private static Magazine m_magazine;
  private boolean m_getBallInQueue;
  private int m_getBallsInMag;

  public RefillQueue(Queue queue, Magazine magazine) {
    m_queue = queue;
    m_magazine = magazine;
    addRequirements(m_queue);
    addRequirements(m_magazine);
  }

  @Override
  public void execute() {
    m_getBallInQueue = m_queue.getBallInQueue();
    m_getBallsInMag = m_magazine.countBall();
    if (!m_getBallInQueue && m_getBallsInMag > 0) {
      m_queue.run(0.5, false);
      m_magazine.setSpeed(0.5);
    }
  }

  @Override
  public boolean isFinished() {
    return !SmartDashboard.getBoolean("invalid ball count", false);
  }
}