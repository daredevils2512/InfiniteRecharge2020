package frc.robot.vision;

public enum Limelight {
  Turret("limelight", 0);

  private final String m_networkTableName;
  private final double m_angleOfElevation;

  private Limelight(String networkTableName, double angleOfElevation) {
    m_networkTableName = networkTableName;
    m_angleOfElevation = angleOfElevation;
  }

  public String getNetworkTableName() {
    return m_networkTableName;
  }

  public double getAngleOfElevation() {
    return m_angleOfElevation;
  }
}
