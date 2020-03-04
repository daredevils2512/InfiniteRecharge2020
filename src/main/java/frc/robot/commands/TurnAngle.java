package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.CommandLogger;
import frc.robot.utils.DareMathUtil;

public class TurnAngle extends CommandLogger {
  private final Drivetrain m_drivetrain;
  private final double m_angle;

  private final PIDController m_angleController;
  private final double m_pGain = 1;
  private final double m_iGain = 0;
  private final double m_dGain = 0;
  private final double m_angleTolerance = 10; // Tolerance in degrees
  private final double m_angularSpeedTolerance = 2; // Tolerance in degrees per second;

  private double m_startAngle;
  private double m_targetAngle;

  public TurnAngle(Drivetrain drivetrain, double angle) {
    m_drivetrain = drivetrain;
    m_angle = angle;
    m_angleController = new PIDController(m_pGain, m_iGain, m_dGain);
    m_angleController.setTolerance(m_angleTolerance, m_angularSpeedTolerance);
    m_angleController.enableContinuousInput(-180, 180);
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    m_startAngle = getWrappedHeading();
    m_targetAngle = m_startAngle + m_angle;
    m_angleController.setSetpoint(m_targetAngle);
  }

  @Override
  public void execute() {
    double angleControllerOutput = m_angleController.calculate(getWrappedHeading());
    m_drivetrain.simpleArcadeDrive(0, angleControllerOutput);
  }

  @Override
  public boolean isFinished() {
    return m_angleController.atSetpoint();
  }

  private double getWrappedHeading() {
    return DareMathUtil.wrap(m_drivetrain.getHeading(), -180, 180);
  }
}