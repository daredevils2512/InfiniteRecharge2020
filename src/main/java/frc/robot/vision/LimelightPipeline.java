package frc.robot.vision;

public enum LimelightPipeline {
  Turret(Limelight.Turret, 0);

  private final Limelight m_limelight;
  private final int m_id;

  private LimelightPipeline(Limelight limelight, int id) {
    m_limelight = limelight;
    m_id = id;
  }

  public Limelight getLimelight() {
    return m_limelight;
  }

  public int getID() {
    return m_id;
  }

  public String getNetworkTableName() {
    return m_limelight.getNetworkTableName();
  }
}
