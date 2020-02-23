package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightUtil {
  private LimelightUtil() {
    
  }

  public static boolean hasTarget(LimelightPipeline pipeline) {
    usePipeline(pipeline);
    return getNetworkTable(pipeline).getEntry("tv").getNumber(0).intValue() == 1;
  }

  public static double getHorizontalOffset(LimelightPipeline pipeline) {
    usePipeline(pipeline);
    return getNetworkTable(pipeline).getEntry("tx").getNumber(0).doubleValue();
  }

  public static double getVerticalOffset(LimelightPipeline pipeline) {
    usePipeline(pipeline);
    return getNetworkTable(pipeline).getEntry("ty").getNumber(0).doubleValue();
  }

  public static double getFill(LimelightPipeline pipeline) {
    usePipeline(pipeline);
    return getNetworkTable(pipeline).getEntry("ta").getNumber(0).doubleValue() / 100;
  }

  public static double getDistance(LimelightPipeline pipeline) {
    usePipeline(pipeline);
    double verticalOffset = getVerticalOffset(pipeline);
    double targetAngleOfElevation = pipeline.getLimelight().getAngleOfElevation() + verticalOffset;
    return verticalOffset / Math.tan(Math.toRadians(targetAngleOfElevation));
  }

  private static void usePipeline(LimelightPipeline pipeline) {
    getNetworkTable(pipeline).getEntry("pipeline").setNumber(pipeline.getID());
  }

  private static NetworkTable getNetworkTable(LimelightPipeline pipeline) {
    return NetworkTableInstance.getDefault().getTable(pipeline.getNetworkTableName());
  }
}
