/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.interfaces.IDrivetrain;
import frc.robot.subsystems.interfaces.ITurret;
import frc.robot.vision.Limelight;

/**
 * Add your docs here.
 */
public class HexagonPosition {
    private final IDrivetrain m_drivetrain;
    private final ITurret m_turret;
    private final Limelight m_limelight;
    private final NetworkTable m_networkTable;
    private final double m_tolerance = 5; //probabl shouldnt be here but idk seems fine for now

    private double m_turretPosition;
    private double m_robotPosition;

    public HexagonPosition(IDrivetrain drivetrain, ITurret turret, Limelight limelight) {
        m_drivetrain = drivetrain;
        m_turret = turret;
        m_limelight = limelight;
        m_networkTable = NetworkTableInstance.getDefault().getTable("hexagon position");
    }

    private void calculatePosition() {
        m_turretPosition = m_limelight.hasTarget() ? m_turret.getAngle() + m_limelight.getLastPosition() : m_turretPosition;
        m_robotPosition = m_limelight.hasTarget() ? (m_drivetrain.getHeading()) + m_limelight.getLastPosition() + m_turret.getAngle() : m_robotPosition;
    }

    public void updatePosition() {
        calculatePosition();
        m_networkTable.getEntry("robot relative position").setDouble(getRobotRelativePosition());
        m_networkTable.getEntry("turret relative position").setDouble(getTurretRelativePosition());
    }
    
    private double getRobotRelativePosition() {
        return m_robotPosition;
    }

    private double getTurretRelativePosition() {
        return m_robotPosition - m_drivetrain.getHeading();
    }
}
