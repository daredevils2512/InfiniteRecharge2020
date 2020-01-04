/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final int m_leftDrive1ID = 1;
  private final int m_leftDrive2ID = 2;
  private final int m_rightDrive1ID = 3;
  private final int m_rightDrive2ID = 4;
  private final int m_shifterForwardChannel = 0;
  private final int m_shifterReverseChannel = 1;
  private final DoubleSolenoid.Value m_highGearValue = Value.kForward;
  private final DoubleSolenoid.Value m_lowGearValue = Value.kReverse;

  private final WPI_TalonFX m_leftDrive1;
  private final WPI_TalonFX m_leftDrive2;
  private final WPI_TalonFX m_rightDrive1;
  private final WPI_TalonFX m_rightDrive2;

  private final DifferentialDrive m_differentialDrive;

  private final DoubleSolenoid m_shifter;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    m_leftDrive1 = new WPI_TalonFX(m_leftDrive1ID);
    m_leftDrive2 = new WPI_TalonFX(m_leftDrive2ID);
    m_rightDrive1 = new WPI_TalonFX(m_rightDrive1ID);
    m_rightDrive2 = new WPI_TalonFX(m_rightDrive2ID);

    m_leftDrive2.follow(m_leftDrive1);
    m_rightDrive2.follow(m_rightDrive1);

    m_differentialDrive = new DifferentialDrive(m_leftDrive1, m_rightDrive1);

    m_shifter = new DoubleSolenoid(m_shifterForwardChannel, m_shifterReverseChannel);
  }

  @Override
  public void periodic() {
    
  }

  public void arcadeDrive(double move, double turn) {
    m_differentialDrive.arcadeDrive(move, turn);
  }

  public void setLowGear(boolean wantsLowGear) {
    m_shifter.set(wantsLowGear ? m_lowGearValue : m_highGearValue);
  }

  public boolean getLowGear() {
    return m_shifter.get() == m_lowGearValue;
  }
}
