package frc.robot.utilities;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoBuilder {
    private final Drivetrain m_drivetrain;

    public AutoBuilder(String name, Drivetrain drivetrain, PIDConstants translationConstants, PIDConstants rotationConstants) {
        m_drivetrain = drivetrain;
    }

}
