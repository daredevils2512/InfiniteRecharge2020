/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Limelight manager for power cell target tracking
 */
public class Limelight {
  // Center is (0,0)
  public static final double RANGE_X_DEGREES = 29.8;
  public static final double RANGE_Y_DEGREES = 24.85;
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
    lastPostion = 1.0;
  }

  public Pipeline getDefaultPipeline() {
    return m_pipeline;
  }

  //Limelight table getters

  public void setPipeline(Pipeline pipeline) {
    m_table.getEntry("pipeline").setNumber(pipeline.getID());
  }

  public boolean hasTarget() {
    return m_table.getEntry("tv").getNumber(0).intValue() == 1;
  }

  public double tx() {
    return m_table.getEntry("tx").getNumber(0).doubleValue();
  }

  public double ty() {
    return m_table.getEntry("ty").getNumber(0).doubleValue();
  }

  public double ta() {
    return m_table.getEntry("ta").getNumber(0).doubleValue();
  }

  public double ts() {
    return m_table.getEntry("ts").getNumber(0).doubleValue();
  }

  public double tl() {
    return m_table.getEntry("tl").getNumber(0).doubleValue();
  }

  public int tshort() {
    return m_table.getEntry("tshort").getNumber(0).intValue();
  }

  public int tlong() {
    return m_table.getEntry("tlong").getNumber(0).intValue();
  }

  public int thor() {
    return m_table.getEntry("thor").getNumber(0).intValue();
  }

  public int tvert() {
    return m_table.getEntry("tvert").getNumber(0).intValue();
  }

  public double getLastPosition() {
    if (tx() != 0) { 
      lastPostion = tx(); 
    }
    return lastPostion;
  }
}
