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
  public enum Pipeline {
    PowerCellTopTarget(2),
    PowerCellsLimelight(1),
    PowerCells(0);

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

  public Limelight() {
    m_table = NetworkTableInstance.getDefault().getTable("limelight");
    this.lastPostion = 1.0;
  }

  //Limelight table getters

  public void setPipeline(Pipeline pipeline) {
    m_table.getEntry("pipeline").setNumber(pipeline.getID());
  }

  public boolean hasTarget() {
    return m_table.getEntry("tv").getDouble(0) == 1;
  }

  public double tx() {
    return m_table.getEntry("tx").getDouble(0);
  }

  public double ty() {
    return m_table.getEntry("ty").getDouble(0);
  }

  public double ta() {
    return m_table.getEntry("ta").getDouble(0);
  }

  public double ts() {
    return m_table.getEntry("ts").getDouble(0);
  }

  public double tl() {
    return m_table.getEntry("tl").getDouble(0);
  }

  public double getLastPosition() {
    if (tx() != 0) { 
      lastPostion = tx(); 
    }
    return lastPostion;
  }
}
