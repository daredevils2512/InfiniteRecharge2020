package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.subsystems.interfaces.IPiTable;

public class PiTable implements IPiTable {

    private NetworkTable m_table;

    private double[] defaultValue = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
 
    public PiTable() {
        m_table = NetworkTableInstance.getDefault().getTable("ball table");
    }

    public boolean hasTarget() {
        return m_table.getSubTable("info").getEntry("has target").getBoolean(false);
    }

    public int getNumberOfTargets() {
        return (int)m_table.getSubTable("info").getEntry("targets").getDouble(0.0);
    }

    /**
     * x offset in degrees
     * <p> really just entry[0] but like this for verbose or somehting
     * @param entry a double array returned by one of the getters
     * @return the index zero of the entry
     */
    public double getXOffset(double[] entry) {
        return entry[0];
    }
    
    /**
     * y offset in degrees
     * <p> really just entry[1] but like this for verbose or somehting
     * @param entry a double array returned by one of the getters
     * @return the index one of the entry
     */
    public double getYOffset(double[] entry) {
        return entry[1];
    }

    /**
     * distance in meters
     * <p> really just entry[2] but like this for verbose or somehting
     * @param entry a double array returned by one of the getters
     * @return the index two of the entry
     */
    public double getDistance(double[] entry) {
        return entry[2];
    }

    /**
     * distance straight forward to how ever far forward the ball is in meters
     * <p> really juset entry[3] but better
     * @param entry a double array of one of the balls
     * @return the index three of the array
     */
    public double getX(double[] entry) {
        return entry[3];
    }

    /**
     * distance straight sideways to how ever far sideways the ball is in meters
     * <p> really juset entry[4] but better
     * @param entry a double array of one of the balls
     * @return the index four of the array
     */
    public double getY(double[] entry) {
        return entry[4];
    }

    /**
     * size of target in pixels 
     * <p> not too useful but its one of the things returned by the pipeline so why not
     * <p> really just entry[3] but like this for verbose or somehting
     * @param entry a double array returned by one of the getters
     * @return the index three of the entry
     */
    public double getSize(double[] entry) {
        return entry[5];
    }

    public Pose2d getPose(double[] entry) {
        return new Pose2d(new Translation2d(getX(entry), getY(entry)), new Rotation2d(Math.toRadians(getXOffset(entry))));
    }

    /**
     * gets closest target
     * @return double array {0.0, 0.0, 0.0, 0.0} if no target found or if the tables screwed up
     */
    public double[] getClosestTarget() {
        if (m_table.getSubTable("info").getEntry("closest target").getString("ball 0") != "ball 0") {
            String closestTarget = m_table.getSubTable("info").getEntry("closest target").getString("ball 1");
            return m_table.getEntry(closestTarget).getDoubleArray(defaultValue);
        } else {
            return defaultValue;
        }
    }

    public double[] getTarget(int ball) {
        return m_table.getEntry("ball " + ball).getDoubleArray(defaultValue);
    }

    public double[] getLastClosestTarget() {
        double[] lastClosestTargetPosition = defaultValue;
        if (hasTarget()) {
            lastClosestTargetPosition = getClosestTarget();
        }
        return lastClosestTargetPosition;
    }

    /**
     * mainly for convience
     * @return the pose of the closest ball
     */
    public Pose2d getClosestBallPose() {
        return getPose(getClosestTarget());
    }
}