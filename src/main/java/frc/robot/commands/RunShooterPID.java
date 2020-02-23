package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.interfaces.IShooter;

public class RunShooterPID extends CommandBase{

    private IShooter m_shooter;

    private Supplier<Double> m_speed;

    public RunShooterPID(IShooter shooter, Supplier<Double> speed) {
        m_shooter = shooter;
        m_speed = speed;    
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        m_shooter.setTargetVelocity(m_speed.get());

    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setPercentOutput(0.0);
    }
}