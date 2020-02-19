package frc.robot.vision;

public enum Limelight {
  Turret("limelight");

  private final String m_networkTableName;

  private Limelight(String networkTableName) {
    m_networkTableName = networkTableName;
  }

  public String getNetworkTableName() {
    return m_networkTableName;
  }
}
