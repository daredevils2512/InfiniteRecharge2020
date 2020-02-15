package frc.robot.commands;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoCommand extends CommandBase {

    public Drivetrain m_drivetrain;

    public AutoCommand(Drivetrain drivetrain, String pathToPath) {
        m_drivetrain = drivetrain;
        Path path = Filesystem.getDeployDirectory().toPath().resolve(pathToPath);
        try {
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(path);
        } catch(IOException e) {
            e.printStackTrace();
        }
    }
}