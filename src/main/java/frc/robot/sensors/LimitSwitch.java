/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Limit switch wrapper class
 */
public class LimitSwitch extends DevilDigitalInput {

    /**
     * Create a new limit switch
     * @param port digital input port
     */
    public LimitSwitch(int port) {
        super(port);
    }
}
