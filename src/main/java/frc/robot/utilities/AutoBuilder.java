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
    private final SwerveAutoBuilder m_defaultBuilder;

    public AutoBuilder(Drivetrain drivetrain, PIDConstants translationConstants, PIDConstants rotationConstants, HashMap<String, Command> events) {
        m_drivetrain = drivetrain;
        m_defaultBuilder = new SwerveAutoBuilder(m_drivetrain::getPose, m_drivetrain::resetOdometry, DriveConstants.kDriveKinematics, translationConstants, rotationConstants, m_drivetrain::setModuleStates, events, true, m_drivetrain);
    }

    public CommandBase fullAuto(String name) {
        List<PathPlannerTrajectory> defaultTrajectories = PathPlanner.loadPathGroup(name, new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAcceleration));
        PathConstraints firstConstraints = new PathConstraints(Math.min(Math.max(defaultTrajectories.get(0).getInitialState().velocityMetersPerSecond, defaultTrajectories.get(0).getEndState().velocityMetersPerSecond), DriveConstants.kMaxSpeedMetersPerSecond), DriveConstants.kMaxAcceleration);
        PathConstraints[] correctedConstraints = new PathConstraints[defaultTrajectories.size() - 1];
        for (int i = 1; i < defaultTrajectories.size(); i++) {
            correctedConstraints[i] = new PathConstraints(Math.min(Math.max(defaultTrajectories.get(i).getInitialState().velocityMetersPerSecond, defaultTrajectories.get(i).getEndState().velocityMetersPerSecond), DriveConstants.kMaxSpeedMetersPerSecond), DriveConstants.kMaxAcceleration);
        }
        return m_defaultBuilder.fullAuto(PathPlanner.loadPathGroup(name, firstConstraints, correctedConstraints));
    }

}
