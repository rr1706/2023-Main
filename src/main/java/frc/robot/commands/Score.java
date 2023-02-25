package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.StateConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.MotionControlSystem;

public class Score extends SequentialCommandGroup {
    private final Drivetrain m_drivetrain;
    private final PoseEstimator m_poseEstimator;
    private final Limelight m_vision;
    private final Claw m_claw;
    private final MotionControlSystem m_motionControl;
    private final PIDCommand m_aimPID;
    private final PIDCommand m_movePID;
    private final double m_goalPosition;
    private final int m_goalHeight;

    public Score(Drivetrain drivetrain, PoseEstimator poseEstimator, Limelight vision, MotionControlSystem motionControl, Claw claw, double goalPosition, int goalHeight) {
        m_drivetrain = drivetrain;
        m_poseEstimator = poseEstimator;
        m_vision = vision;
        m_claw = claw;
        m_motionControl = motionControl;
        m_goalPosition = goalPosition;
        m_goalHeight = goalHeight;
        m_aimPID = new PIDCommand(
            new PIDController(0.05, 0.0, 0.0),
            m_vision::getTX,
            0.0,
            this::adjust,
            m_drivetrain, m_vision
        );
        m_movePID = new PIDCommand(
            new PIDController(0.25, 0.0, 0.0),
            m_poseEstimator.getPose()::getY,
            goalPosition,
            this::adjust,
            m_drivetrain, m_poseEstimator
        );

        if (m_drivetrain.getChassisSpeed().equals(new ChassisSpeeds())) {
            if (m_poseEstimator.inside(FieldConstants.kScoringZone, true)) {
                if (!((m_poseEstimator.getPose().getY() >= goalPosition - FieldConstants.kScoringTolerance) && (m_poseEstimator.getPose().getY() <= goalPosition + FieldConstants.kScoringTolerance))) {
                    addCommands(
                        m_drivetrain.toPose(m_poseEstimator.getPose(), new Pose2d(2.30, m_poseEstimator.getPose().getY(), m_poseEstimator.getPose().getRotation()), m_poseEstimator::getPose),
                        m_movePID
                    ); 
                }
            } else if (m_poseEstimator.inside(FieldConstants.kScoringPrepZone, false)) {
                addCommands(
                    m_movePID
                );
            }

            addCommands(
                m_aimPID,
                new InstantCommand(() -> m_motionControl.setState(StateConstants.kShoot)).andThen(new WaitUntilCommand(m_motionControl::isFinished)),
                new InstantCommand(() -> m_claw.setSpeed(100)).andThen(new WaitCommand(0.2))
            );
        }
    }

    private void adjust(double pidOutput) {
        m_drivetrain.drive(0.0, pidOutput, 0.0, true, true);
    }
}
