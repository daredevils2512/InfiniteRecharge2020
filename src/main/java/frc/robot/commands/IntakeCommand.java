/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.IMagazine;
import frc.robot.utils.CommandLogger;
import frc.robot.utils.MagazinePowerCellCounter;

/**
 * Add your docs here.
 */
public class IntakeCommand extends CommandLogger {

  private double m_runIntake;
  private double m_runMagazine;
  private double m_magazineSpeed;
  private double m_intakeSpeed;
  private IIntake m_intake;
  private IMagazine m_magazine;
  private Supplier<Double> m_intakeAxis;
  private Supplier<Double> m_extenderSpeedSupplier;
  private double m_extenderMaxSpeed;
  private Supplier<Boolean> m_extendedSupplier;
  private Supplier<Boolean> m_shouldExtendSupplier;

  private boolean magazineRunState = false;

  public IntakeCommand(IIntake intake, Supplier<Double> intakeAxis, double intakeSpeed, IMagazine magazine,
      double magazineSpeed, Supplier<Double> extenderSpeed, double extenderMaxSpeed, Supplier<Boolean> extended,
      Supplier<Boolean> shouldExtend) {
    m_intake = intake;
    m_intakeAxis = intakeAxis;
    m_magazine = magazine;
    m_intakeSpeed = intakeSpeed;
    m_magazineSpeed = magazineSpeed;
    m_extenderSpeedSupplier = extenderSpeed;
    m_extenderMaxSpeed = extenderMaxSpeed;
    m_extendedSupplier = extended;
    m_shouldExtendSupplier = shouldExtend;
    addRequirements(intake);

    m_runMagazine = 0.0;
    m_runIntake = 0.0;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_runIntake = m_intakeAxis.get() * m_intakeSpeed;

    if (
      //MagazinePowerCellCounter.getCount() <= 3 && 
        m_intakeAxis.get() != 0.0) {
      m_runMagazine = m_magazineSpeed;
      magazineRunState = true;
      m_magazine.setSpeed(m_runMagazine);
    } else if(magazineRunState) {
      m_logger.info("stopped magazine");
      m_runMagazine = 0.0;
      m_magazine.setSpeed(m_runMagazine);
      magazineRunState = false;
    }

    SmartDashboard.putNumber("intake set speed", m_runIntake);
    if (m_shouldExtendSupplier.get()) {
      m_intake.setMotionMagicEnabled(true);
      if (m_extendedSupplier.get()) m_intake.extend();
      else m_intake.retract();
    } else {
      m_intake.setMotionMagicEnabled(false);
      m_intake.runExtender(m_extenderSpeedSupplier.get() * m_extenderMaxSpeed);
    }
    m_intake.runIntake(m_runIntake);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) m_logger.info("interuppted");
    m_intake.runIntake(0.0);
    m_magazine.setSpeed(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
