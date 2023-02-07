package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class Docking extends CommandBase {
    private final Drivetrain m_drive;
    private final Pose2d m_pose;
    private final PIDCommand levelingPID;

    public Docking(Drivetrain drive) {
        m_drive = drive;
        m_pose = m_drive.getPose();
        levelingPID = new PIDCommand(new PIDController(0, 0, 0), null, 0, null, m_drive);
    }

    

}
