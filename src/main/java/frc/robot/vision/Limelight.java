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
    PowerCellTopTarget(0),
    PowerCells(1);

    private int m_id;

    private Pipeline(int id) {
      m_id = id;
    }

    public int getID() {
      return m_id;
    }
  }

  private NetworkTable table;

  public Limelight() {

  }

  @SuppressWarnings("unused")
  private void setPipeline(Pipeline pipeline) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline.getID());
  }

  @SuppressWarnings("unused")
  private boolean hasTarget() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;
  }

  @SuppressWarnings("unused")
  private double tx() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  @SuppressWarnings("unused")
  private double ty() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  @SuppressWarnings("unused")
  private double ta() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  }

  @SuppressWarnings("unused")
  private double ts() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
  }

  @SuppressWarnings("unused")
  private double tl() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0);
  }
}
