package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.interfaces.IQueue;
import frc.robot.utils.DareMathUtil;

public class ManualRunQueue extends RunQueueCommand {

    private IQueue m_queue;
    private double speed;

    private NetworkTable m_networkTable;
    private final NetworkTableEntry m_shooterVelocity;

    public ManualRunQueue(IQueue queue, double speed) {
        super(queue, speed);
        m_queue = queue;
        this.speed = speed;
        m_networkTable = NetworkTableInstance.getDefault().getTable(this.getName());
        m_shooterVelocity = m_networkTable.getEntry("Shooter RPM");
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

    @Override
    protected boolean shouldRunQueue() {
        boolean shouldRunQueue = m_queue.hasPowerCell()
            && DareMathUtil.isWithinXOf(m_networkTable.getEntry("Shooter RPM").getDouble(0),
             m_networkTable.getEntry("shooter target velocity").getDouble(0),
             50);
        return shouldRunQueue;
    }
}