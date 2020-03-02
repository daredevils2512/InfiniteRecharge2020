/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.logging.*;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Compressor;
import frc.robot.subsystems.interfaces.ICompressorManager;

/**
 * Add your docs here.
 */
public class CompressorManager extends LoggingSubsystem implements ICompressorManager {
  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_isRunningEntry;
  private final NetworkTableEntry m_closedLoopControlEntry;

  private Compressor m_compressor = new Compressor();

  public CompressorManager() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_isRunningEntry = m_networkTable.getEntry("Is running");
    m_closedLoopControlEntry = m_networkTable.getEntry("closed loop contorl");
  
  }

  @Override
  public void periodic() {
    m_isRunningEntry.setBoolean(m_compressor.enabled());
    m_closedLoopControlEntry.setBoolean(getClosedLoopControl());
  }

  @Override
  public void setClosedLoopControl(boolean wantsClosedLoopControl) {
    m_compressor.setClosedLoopControl(wantsClosedLoopControl);
  }

  @Override
  public boolean getClosedLoopControl() {
    return m_compressor.getClosedLoopControl();
  }
  
  @Override
  public void toggleCompressor() {
    m_compressor.setClosedLoopControl(!getClosedLoopControl());
    m_logger.fine("Compressor closed loop control: " + getClosedLoopControl());
  }
}
