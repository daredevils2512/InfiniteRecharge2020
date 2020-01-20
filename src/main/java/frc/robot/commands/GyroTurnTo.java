package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class GyroTurnTo extends CommandBase {
  private final double m_toAngle;
  private final double m_threshold;
  private final double m_maxSpeed;
  private final double kTurn = 0.1;

  private final Drivetrain m_drivetrain;

  private boolean isFinished = false;

  public GyroTurnTo(Drivetrain drivetrain, final double toAngle, final double threshold, final double maxSpeed) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
    m_toAngle = toAngle;
    m_threshold = threshold;
    m_maxSpeed = maxSpeed;
  }

  @Override
  public void initialize() {
    m_drivetrain.resetGyro();
  }

  @Override
  public void execute() {
    //might not turn the right way
    //also not done yet
    if (m_toAngle >= m_drivetrain.getYaw() + m_threshold) {
      // m_drivetrain.arcadeDrive(0, Math.min(m_maxSpeed, ));
    } else if (m_toAngle <= m_drivetrain.getYaw() - m_threshold) {
      // m_drivetrain.arcadeDrive(0, Math.max(m_maxSpeed, ));
    } else {
      isFinished = true;
    }
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

}