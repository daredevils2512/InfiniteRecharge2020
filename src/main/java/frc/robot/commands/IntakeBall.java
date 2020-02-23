/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.IMagazine;
import frc.robot.utils.MagazinePowerCellCounter;

public class IntakeBall extends CommandBase {

  private final IIntake m_intake;
  private final double m_intakeSpeed;
  private final IMagazine m_magazine;
  private final double m_magazineSpeed;
  private final MagazinePowerCellCounter m_counter;

  private int m_startingCount;
  private double m_runIntake = 0.0;
  private double m_runMagazine = 0.0;

  /**
   * Creates a new IntakeBall.
   */
  public IntakeBall(IIntake intake, double intakeSpeed, IMagazine magazine, double magazineSpeed, MagazinePowerCellCounter counter) {
    m_intake = intake;
    m_intakeSpeed = intakeSpeed;
    m_magazine = magazine;
    m_magazineSpeed = magazineSpeed;
    m_counter = counter;

    addRequirements(intake, magazine);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startingCount = m_counter.getCount();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_runIntake = 0.0;
    m_runMagazine = 0.0;
    if (m_startingCount == 0) {
      m_runIntake = m_intakeSpeed;
      m_runMagazine = m_magazineSpeed;
    } else {
      if (m_magazine.getPowerCellDetected()) {
        m_runMagazine = m_magazineSpeed;
        m_runIntake = m_intakeSpeed;
      } else {
        m_runIntake = m_intakeSpeed;
      }
    }
    m_intake.runIntake(m_runIntake);
    m_magazine.setSpeed(m_runMagazine);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.runIntake(0.0);
    m_magazine.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_counter.getCount() > m_startingCount;
  }
}
