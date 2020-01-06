/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Intake;

/**
 * Definition for all commands
 */
public final class Commands {
    private static final class IntakeCommand extends CommandBase {
        private final Intake m_intake;

        public IntakeCommand(Intake intake) {
            m_intake = intake;
            addRequirements(m_intake);
        }

        @Override
        public void initialize() {
            m_intake.setExtended(true);
        }

        @Override
        public void execute() {
            m_intake.run(1);
        }

        @Override
        public void end(boolean interrupted) {
            m_intake.run(0);
            m_intake.setExtended(false);
        }
    }

    private Commands() {

    }

    /**
     * Extends and starts running the power cell intake
     * @return New {@link Command}
     */
    public static Command startIntaking(Intake intake) {
        return new InstantCommand(() -> intake.setExtended(true), intake)
            .andThen(new RunCommand(() -> intake.run(1), intake));
    }

    /**
     * Stops running and retracts the power cell intake
     * @return New {@link Command}
     */
    public static Command stopIntaking(Intake intake) {
        return new InstantCommand(() -> intake.run(0), intake)
            .andThen(() -> intake.setExtended(false), intake);
    }

    /**
     * Extends and starts running the power cell intake
     * 
     * <p>Stops running and retracts intake when interrupted
     * @return New {@link Command}
     */
    public static Command intake(Intake intake) {
        return new IntakeCommand(intake);
    }
}
