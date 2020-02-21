package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.interfaces.IDrivetrain;

public final class AccelerationLimitedVelocityArcadeDrive extends CommandBase {
  private final double m_maxAcceleration;
  private final double m_maxAngularAcceleration;

  private final IDrivetrain m_drivetrain;
  private final DoubleSupplier m_moveSupplier;
  private final DoubleSupplier m_turnSupplier;
  private final SlewRateLimiter m_accelerationLimiter;
  private final SlewRateLimiter m_angularAccelerationLimiter;

  /**
   * Velocity arcade drive with acceleration control
   * @param drivetrain Drivetrain to use
   * @param moveSupplier Forward speed supplier (-1 to +1)
   * @param turnSupplier Turn speed supplier (-1 to +1)
   * @param maxAcceleration Units per second of move supplier
   * @param maxAngularAcceleration Units per second of turn supplier
   */
  public AccelerationLimitedVelocityArcadeDrive(IDrivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier, double maxAcceleration, double maxAngularAcceleration) {
    m_maxAcceleration = maxAcceleration;
    m_maxAngularAcceleration = maxAngularAcceleration;
    m_drivetrain = drivetrain;
    m_moveSupplier = moveSupplier;
    m_turnSupplier = turnSupplier;
    m_accelerationLimiter = new SlewRateLimiter(m_maxAcceleration);
    m_angularAccelerationLimiter = new SlewRateLimiter(m_maxAngularAcceleration);
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    double move = m_accelerationLimiter.calculate(m_moveSupplier.getAsDouble());
    double turn = m_angularAccelerationLimiter.calculate(m_turnSupplier.getAsDouble());
    m_drivetrain.velocityArcadeDrive(move * m_drivetrain.getMaxSpeed(), turn * m_drivetrain.getMaxAngularSpeed());
  }
}