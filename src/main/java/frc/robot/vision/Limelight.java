/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.interfaces.ILimelight;

/**
 * returned 228 in at 208 inches
 */

/**
 * <h1> Limelight manager for power cell target tracking
 */
public class Limelight implements ILimelight {
  private static Logger logger = Logger.getLogger(Limelight.class.getName());
  // Center is (0,0)
  public static final double RANGE_X_DEGREES = 29.8;
  public static final double RANGE_Y_DEGREES = 24.85;
  private final double m_angle = 20;
  private final double m_heightOffset = 98.25 - 19.25;
  private final Pipeline m_pipeline;

  public enum Pipeline {
    PowerCellTopTarget(2),
    PowerCellsLimelight(1),
    PowerCells(0),
    Hexagon(3);

    private int m_id;

    private Pipeline(int id) {
      m_id = id;
    }

    public int getID() {
      return m_id;
    }
  }

  private NetworkTable m_table;

  private double lastPostion;

  public Limelight(Pipeline defaultPipeline) {
    m_pipeline = defaultPipeline;
    m_table = NetworkTableInstance.getDefault().getTable("limelight");
    this.setPipeline(defaultPipeline);
    lastPostion = 1.0;
  }

  @Override
  public Pipeline getDefaultPipeline() {
    logger.log(Level.FINE, "default pipeline = ", m_pipeline);
    return m_pipeline;
  }

  //Limelight table getters

  @Override
  public void setPipeline(Pipeline pipeline) {
    m_table.getEntry("pipeline").setNumber(pipeline.getID());
  }

  @Override
  public void setLEDMode(LimelightLEDMode ledMode) {
    m_table.getEntry("ledMode").setNumber(ledMode.getIntValue());
  }

  @Override
  public LimelightLEDMode getLEDMode() {
    int intValue = m_table.getEntry("ledMode").getNumber(0).intValue();
    return LimelightLEDMode.fromIntValue(intValue);
  }

  @Override
  public boolean hasTarget() {
    return m_table.getEntry("tv").getNumber(0).intValue() == 1;
  }

  @Override
  public double tx() {
    return m_table.getEntry("tx").getNumber(0).doubleValue();
  }

  @Override
  public double ty() {
    return m_table.getEntry("ty").getNumber(0).doubleValue();
  }

  @Override
  public double ta() {
    return m_table.getEntry("ta").getNumber(0).doubleValue();
  }

  @Override
  public double ts() {
    return m_table.getEntry("ts").getNumber(0).doubleValue();
  }

  @Override
  public double tl() {
    return m_table.getEntry("tl").getNumber(0).doubleValue();
  }

  @Override
  public int tshort() {
    return m_table.getEntry("tshort").getNumber(0).intValue();
  }

  @Override
  public int tlong() {
    return m_table.getEntry("tlong").getNumber(0).intValue();
  }

  @Override
  public int thor() {
    return m_table.getEntry("thor").getNumber(0).intValue();
  }

  @Override
  public int tvert() {
    return m_table.getEntry("tvert").getNumber(0).intValue();
  }

  @Override
  public double getLastPosition() {
    if (tx() != 0) { 
      lastPostion = tx(); 
    }
    logger.log(Level.FINER, "last pos", lastPostion);
    return lastPostion;
  }

  /**
   * 
   * @return distance in units of something to the tagret
   */
  @Override
  public double getDistanceToTarget() {
    return Units.inchesToMeters(m_heightOffset) / Math.tan(Math.toRadians(m_angle + this.ty()));
  }
}
