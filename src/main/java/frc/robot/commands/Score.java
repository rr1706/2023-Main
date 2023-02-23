package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimator;

public class Score extends CommandBase {
    private final Drivetrain m_drivetrain;
    private final PoseEstimator m_poseEstimator;
    private final Limelight m_vision;
    private final PIDCommand m_PID;

    public Score(Drivetrain drivetrain, PoseEstimator poseEstimator, Limelight vision) {
        m_drivetrain = drivetrain;
        m_poseEstimator = poseEstimator;
        m_vision = vision;
        m_PID = new PIDCommand(new PIDController(0, 0, 0),
        m_drivetrain::getTilt,
        0.0,
        this::moveToScore,
        m_drivetrain, m_vision);
    }

    private void moveToScore(double pidOutput) {
        Pose2d currentPose = m_poseEstimator.getPose();
        m_drivetrain.toPose(currentPose, new Pose2d(new Translation2d(currentPose.getX(), currentPose.getY() + pidOutput), currentPose.getRotation()));
    }

}
