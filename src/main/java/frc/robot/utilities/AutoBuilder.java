package frc.robot.utilities;

import java.util.ArrayList;
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
import frc.robot.subsystems.PoseEstimator;

public class AutoBuilder {
    private final Drivetrain m_drivetrain;
    private final PoseEstimator m_poseEstimator;
    private final SwerveAutoBuilder m_defaultBuilder;

    public AutoBuilder(Drivetrain drivetrain, PoseEstimator poseEstimator, PIDConstants translationConstants, PIDConstants rotationConstants, HashMap<String, Command> events) {
        m_drivetrain = drivetrain;
        m_poseEstimator = poseEstimator;
        m_defaultBuilder = new SwerveAutoBuilder(m_poseEstimator::getPose, m_poseEstimator::resetOdometry, translationConstants, rotationConstants, m_drivetrain::setModuleStates, events, m_drivetrain);
    }

    public CommandBase fullAuto(String name) {
        List<PathPlannerTrajectory> defaultTrajectories = PathPlanner.loadPathGroup(name, new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAcceleration));
        ArrayList<PathConstraints> correctedSpeeds = new ArrayList<PathConstraints>();
        for (PathPlannerTrajectory traj : defaultTrajectories) {
            double correctedSpeed = Math.min(Math.max(traj.getInitialState().velocityMetersPerSecond, traj.getEndState().velocityMetersPerSecond), DriveConstants.kMaxSpeedMetersPerSecond);
            correctedSpeeds.add(new PathConstraints(correctedSpeed, DriveConstants.kMaxAcceleration));
        }
        PathConstraints firstConstraints = correctedSpeeds.get(0);
        correctedSpeeds.remove(0);
        return m_defaultBuilder.fullAuto(PathPlanner.loadPathGroup(name, firstConstraints,(PathConstraints[]) correctedSpeeds.toArray()));
    }

}
