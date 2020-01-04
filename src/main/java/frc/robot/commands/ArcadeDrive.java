/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends Drive {
  private final DoubleSupplier m_moveSupplier;
  private final DoubleSupplier m_turnSupplier;

  /**
   * Creates a new ArcadeDrive.
   */
  public ArcadeDrive(Drivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
    super(drivetrain);
    m_moveSupplier = moveSupplier;
    m_turnSupplier = turnSupplier;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_moveSupplier.getAsDouble(), m_turnSupplier.getAsDouble());
  }
}
