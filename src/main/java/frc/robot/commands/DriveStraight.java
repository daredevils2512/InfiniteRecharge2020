package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.interfaces.IDrivetrain;

public class DriveStraight extends CommandBase {
  private final IDrivetrain m_drivetrain;
  private final double m_distance;
  private double m_startPosition;
  private double m_targetPosition;
  private final PIDController m_distanceController;
  private final double m_pGain = 1;
  private final double m_iGain = 0;
  private final double m_dGain = 0;

  public DriveStraight(IDrivetrain drivetrain, double distance) {
    m_drivetrain = drivetrain;
    m_distance = distance;
    m_distanceController = new PIDController(m_pGain, m_iGain, m_dGain);
    m_distanceController.setTolerance(0.01);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_startPosition = averageEncoderDistance();
    m_targetPosition = m_startPosition + m_distance;
    m_distanceController.setSetpoint(m_targetPosition);
  }

  @Override
  public void execute() {
    double distanceControllerOutput = m_distanceController.calculate(averageEncoderDistance());
    m_drivetrain.simpleArcadeDrive(distanceControllerOutput, 0);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.simpleArcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return m_distanceController.atSetpoint();
  }

  private double averageEncoderDistance() {
    return (m_drivetrain.getLeftDistance() + m_drivetrain.getRightDistance()) / 2;
  }
}