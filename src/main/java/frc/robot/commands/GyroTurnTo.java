package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class GyroTurnTo extends CommandBase {
  private final double m_toAngle;
  private final double m_threshold;

  private final Drivetrain m_drivetrain;

  private boolean isFinished = false;

  public GyroTurnTo(Drivetrain drivetrain, final double toAngle, final double threshold) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
    m_toAngle = toAngle;
    m_threshold = threshold;
  }

  @Override
  public void initialize() {
    m_drivetrain.resetGyro();
  }

  @Override
  public void execute() {
    if (m_toAngle >= m_drivetrain.getYaw() + m_threshold) {

    } else if (m_toAngle <= m_drivetrain.getYaw() - m_threshold) {

    } else {
      isFinished = true;
    }
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

}