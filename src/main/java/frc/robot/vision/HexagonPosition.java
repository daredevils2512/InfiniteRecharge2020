/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import java.util.logging.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.interfaces.IDrivetrain;
import frc.robot.subsystems.interfaces.ILimelight;
import frc.robot.subsystems.interfaces.ITurret;
import frc.robot.utils.DareMathUtil;

/**
 * <h6>Add your docs here.
 */
public class HexagonPosition {
    private final IDrivetrain m_drivetrain;
    private final ITurret m_turret;
    private final ILimelight m_limelight;
    private static NetworkTable m_networkTable = NetworkTableInstance.getDefault().getTable("hexagon position");

    private final double m_tolerance = 5.0; // in degrees probaly shouldnt be here but idk whatever

    // {@Link https://www.desmos.com/calculator/6ics7ndnma}
    // new {@Link https://www.desmos.com/calcula;tor/2wbdygdzb3}
    // where distance = x,
    // rpm = ax^2 * bx + c

    private final double a = 57.0356654206; // constatnts
    private final double b = -460.0885934;
    private final double c = 7593.4512926;

    private static double speedBoost = 0.0;

    private double m_turretPosition;
    private double m_robotPosition = 0.0;
    private double m_lastRobotPosition = 0.0;

    public HexagonPosition(IDrivetrain drivetrain, ITurret turret, ILimelight limelight) {
        m_drivetrain = drivetrain;
        m_turret = turret;
        m_limelight = limelight;
        m_networkTable.getEntry("speed boost").setDouble(speedBoost);
    }

    private void calculatePosition() {
        m_networkTable.getEntry("has target").setBoolean(m_limelight.hasTarget());
        m_turretPosition = m_limelight.hasTarget() ? m_turret.getAngle() + m_limelight.tx() : m_turretPosition;
        m_robotPosition = m_limelight.hasTarget() && DareMathUtil.isWithinXOf(m_robotPosition, m_lastRobotPosition, 10)
                ? m_drivetrain.getHeading() + m_limelight.tx() + m_turret.getAngle()
                : m_lastRobotPosition;
        m_lastRobotPosition = m_robotPosition;
    }

    public void updatePosition() {
        speedBoost = m_networkTable.getEntry("speed boost").getDouble(speedBoost);
        calculatePosition();
        m_networkTable.getEntry("calculated shooter rpm").setDouble(calculateShooterSpeed());
        m_networkTable.getEntry("robot relative position").setDouble(getRobotRelativePosition());
        m_networkTable.getEntry("turret relative position").setDouble(getTurretRelativePosition());
    }

    private double getRobotRelativePosition() {
        return m_robotPosition;
    }

    public boolean canShoot() {
        boolean canShoot = DareMathUtil.isWithinXOf(getTurretRelativePosition(), m_turret.getAngle(), m_tolerance);
        m_networkTable.getEntry("can shoot").setBoolean(canShoot);
        return canShoot;
    }

    private double getTurretRelativePosition() {
        return m_robotPosition - m_drivetrain.getHeading();
    }

    private double calculateShooterSpeed() {
        double x = m_limelight.getDistanceToTarget();
        double speed = a * Math.pow(x, 2) + b * x + c;
        m_networkTable.getEntry("unboosted speed").setDouble(speed);
        speed = speed + speedBoost * speed;
        return speed;
    }

    public static void setSpeedBoost(double boostedSpeed, boolean add) {
        Logger.getGlobal().info("boosting speed with " + boostedSpeed + add);
        if (add) {
            speedBoost += boostedSpeed;
        } else {
            speedBoost = boostedSpeed;
        }
        m_networkTable.getEntry("speed boost").setDouble(boostedSpeed);
    }

}