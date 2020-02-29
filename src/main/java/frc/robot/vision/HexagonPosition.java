/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.interfaces.IDrivetrain;
import frc.robot.subsystems.interfaces.ITurret;
import frc.robot.utils.DareMathUtil;
import frc.robot.vision.Limelight;

/**
 * <h1> HHHHHHHHHHHHHHH
 */
public class HexagonPosition {
    private final IDrivetrain m_drivetrain;
    private final ITurret m_turret;
    private final Limelight m_limelight;
    private final NetworkTable m_networkTable;
    private final double m_tolerance = 5.0; //in degrees probaly shouldnt be here but idk whatever

    //constants for ball distance calculation in the form of rpm = ax^s + bx + c where x is distance in meters
    //https://www.desmos.com/calculator/hlfz61fwpw
    private final double a = 180.74488324;
    private final double b = -1491.97857776;
    private final double c = 8764.60835839;

  /**
   * notes for calculations
   * 
   * front of trench: status: good 
   * distance: 4.8
   * set rpm: 6k
   * hood pos: 1900
   * 
   * 6.2 : 6500
   * 5.7 : 6000
   * 2.9 : 6000
   * 4.0 : 5500
   * 4.3 : 5700
   */

    private double m_turretPosition;
    private double m_robotPosition;

    public HexagonPosition(IDrivetrain drivetrain, ITurret turret, Limelight limelight) {
        m_drivetrain = drivetrain;
        m_turret = turret;
        m_limelight = limelight;
        m_networkTable = NetworkTableInstance.getDefault().getTable("hexagon position");
    }

    private void calculatePosition() {
        m_networkTable.getEntry("has target").setBoolean(m_limelight.hasTarget());
        m_turretPosition = m_limelight.hasTarget() ? m_turret.getAngle() + m_limelight.tx() : m_turretPosition;
        m_robotPosition = m_limelight.hasTarget() ? m_drivetrain.getHeading() + m_limelight.tx() + m_turret.getAngle() : m_robotPosition;
    }

    public void updatePosition() {
        calculateShooterSpeed();
        calculatePosition();
        m_networkTable.getEntry("required shooter speed").setDouble(calculateShooterSpeed());
        m_networkTable.getEntry("robot relative position").setDouble(getRobotRelativePosition());
        m_networkTable.getEntry("turret relative position").setDouble(getTurretRelativePosition());
    }
    
    private double getRobotRelativePosition() {
        return m_robotPosition;
    }

    public boolean canShoot() {
        boolean canShoot = DareMathUtil.isWithinXOf(getTurretRelativePosition(), m_turret.getAngle(), m_tolerance);
        m_networkTable.getEntry("can shoot").setBoolean(canShoot);
        return  canShoot;
    }
    
    private double getTurretRelativePosition() {
        return m_robotPosition - m_drivetrain.getHeading();
    }

    /**
     * calculates desired shooter speed using math above
     * {@link HexagonPosition in these}
     * @return
     */
    private double calculateShooterSpeed() {
        double x = m_limelight.getDistanceToTarget();
        return a * Math.pow(x, 2) + b * x + c;
    }

}