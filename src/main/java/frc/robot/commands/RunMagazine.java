/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.interfaces.IMagazine;

/**
 * Add your docs here.
 */
public class RunMagazine extends CommandBase {

    private IMagazine m_magazine;
    private double speed;
    private Logger logger;

    public RunMagazine(IMagazine magazine, double speed) {
        m_magazine = magazine;
        this.speed = speed;
        logger = Logger.getLogger("frc.robot.Magazine.java");
        addRequirements(m_magazine);
    }

    @Override
    public void execute() {
        logger.log(Level.FINE, "setting mag speed to = ", speed);
        m_magazine.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        logger.fine("mag command interrupted");
        m_magazine.setSpeed(0.0);
    }
}