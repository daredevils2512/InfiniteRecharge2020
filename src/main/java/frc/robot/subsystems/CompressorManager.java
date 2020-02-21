/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.logging.*;

import edu.wpi.first.wpilibj.Compressor;
import frc.robot.subsystems.interfaces.ICompressorManager;

/**
 * Add your docs here.
 */
public class CompressorManager implements ICompressorManager {
  private static Logger logger = Logger.getLogger(CompressorManager.class.getName());
  private Compressor compressor = new Compressor();
  private boolean m_compressorEnabled; 
  
  @Override
  public void toggleCompressor(){
    m_compressorEnabled = !m_compressorEnabled;
    logger.log(Level.FINE, "compressor = ", m_compressorEnabled);
    compressor.setClosedLoopControl(m_compressorEnabled);
  } 
}
