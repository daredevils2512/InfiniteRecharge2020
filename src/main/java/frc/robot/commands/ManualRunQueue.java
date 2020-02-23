package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.interfaces.IQueue;

public class ManualRunQueue extends CommandBase {

    private IQueue m_queue;
    private double speed;

    public ManualRunQueue(IQueue queue, double speed) {
        m_queue = queue;
        this.speed = speed;
        addRequirements(m_queue);
    }

    @Override
    public void execute() {
        m_queue.run(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_queue.run(0.0);
    }
}