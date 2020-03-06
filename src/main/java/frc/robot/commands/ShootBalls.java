package frc.robot.commands;

import java.util.logging.Level;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.interfaces.IMagazine;
import frc.robot.subsystems.interfaces.IQueue;
import frc.robot.subsystems.interfaces.IShooter;
import frc.robot.utils.CommandLogger;
import frc.robot.utils.MagazinePowerCellCounter;

public class ShootBalls extends CommandLogger {

    private IShooter m_shooter;
    private IQueue m_queue;
    private double m_queueSpeed;
    private IMagazine m_magazine;
    private double m_magazineSpeed;
    private int m_balls;
    private int m_startingBallCount;
    private int m_finalBallCount;

    public ShootBalls(IShooter shooter, IQueue queue, double queueSpeed, IMagazine magazine, double magazineSpeed, int balls) {
        m_shooter = shooter;
        m_queue = queue;
        m_queueSpeed = queueSpeed;
        m_magazine = magazine;
        m_magazineSpeed = magazineSpeed;
        m_balls = balls;
    }

    @Override
    public void initialize() {
        m_startingBallCount = MagazinePowerCellCounter.getCount();
        m_finalBallCount = (m_balls > m_startingBallCount) ? 0 : m_startingBallCount - m_balls;
        m_logger.info("initalised shooting for " + m_balls + " balls");
    }

    @Override
    public void execute() {
        m_shooter.setToCalculatedVelocity();
        if (m_shooter.isAtSpeed()) {
            m_logger.fine("running queue and magazine");
            m_queue.run(m_queueSpeed);
            m_magazine.setSpeed(m_magazineSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_magazine.setSpeed(0.0);
        m_queue.run(0.0);
    }

    @Override
    public boolean isFinished() {
        return MagazinePowerCellCounter.getCount() <= m_finalBallCount;
    }
    
}