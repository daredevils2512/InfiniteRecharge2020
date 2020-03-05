package frc.robot.commands;

import java.util.function.Supplier;

import frc.robot.subsystems.interfaces.IShooter;
import frc.robot.utils.CommandLogger;

public class RunShooterPID extends CommandLogger {

    private IShooter m_shooter;

    private Supplier<Double> m_speed;

    public RunShooterPID(IShooter shooter, Supplier<Double> speed) {
        m_shooter = shooter;
        m_speed = speed;    
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        m_logger.fine("running");
        m_shooter.setTargetVelocity(m_speed.get());

    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setPercentOutput(0.0);
    }
}