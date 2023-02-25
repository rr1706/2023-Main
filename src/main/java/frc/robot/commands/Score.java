package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private final int m_goalPosition;
    private final int m_goalHeight;

    public Score(Drivetrain drivetrain, PoseEstimator poseEstimator, Limelight vision, MotionControlSystem motionControl, Claw claw, int goalPosition, int goalHeight) {
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
            new PIDController(0.20, 0.0, 0.0),
            m_poseEstimator.getPose()::getY,
            FieldConstants.kScoringPositions[m_goalPosition],
            this::adjust,
            m_drivetrain, m_poseEstimator
        );
        m_movePID.getController().setTolerance(FieldConstants.kScoringTolerance + 0.05);

        if (m_drivetrain.getChassisSpeed().equals(new ChassisSpeeds())) {
            if (m_poseEstimator.inside(FieldConstants.kScoringZone, true) || m_poseEstimator.inside(FieldConstants.kScoringPrepZone, true)) {
                if (!((m_poseEstimator.getPose().getY() >= FieldConstants.kScoringPositions[m_goalPosition] - FieldConstants.kScoringTolerance) && (m_poseEstimator.getPose().getY() <= FieldConstants.kScoringPositions[m_goalPosition] + FieldConstants.kScoringTolerance))) {
                    addCommands(
                        m_drivetrain.toPose(m_poseEstimator.getPose(), new Pose2d(2.30, m_poseEstimator.getPose().getY(), m_poseEstimator.getPose().getRotation()), m_poseEstimator::getPose),
                        m_movePID.alongWith(new WaitUntilCommand(m_movePID.getController()::atSetpoint)),
                        m_aimPID.alongWith(new WaitUntilCommand(m_movePID.getController()::atSetpoint))
                    ); 
                }
            }
            
            addCommands(
                new InstantCommand(() -> setMotionControlState()).alongWith(m_drivetrain.toPose(m_poseEstimator.getPose(), new Pose2d(1.85, m_poseEstimator.getPose().getY(), m_poseEstimator.getPose().getRotation()), m_poseEstimator::getPose)),
                new InstantCommand(() -> m_claw.setSpeed(100)).andThen(new WaitCommand(0.2)),
                new InstantCommand(() -> m_claw.setSpeed(0))
            );
        }
    }

    private void adjust(double pidOutput) {
        m_drivetrain.drive(0.0, pidOutput, 0.0, true, true);
    }

    private void setMotionControlState() {
        if (m_goalHeight == 1) {
            m_motionControl.setState(StateConstants.kLow);
        } else {
            ArrayList<Integer> coneGoals = new ArrayList<>();
            coneGoals.add(1);
            coneGoals.add(3);
            coneGoals.add(4);
            coneGoals.add(6);
            coneGoals.add(7);
            coneGoals.add(9);
            if (coneGoals.contains(m_goalPosition)) {
                if (m_goalHeight == 2) {
                    m_motionControl.setState(StateConstants.kConeMid);
                } else {
                    m_motionControl.setState(StateConstants.kConeHigh);
                }
            } else {
                if (m_goalHeight == 2) {
                    m_motionControl.setState(StateConstants.kCubeMid);
                } else {
                    m_motionControl.setState(StateConstants.kCubeHigh);
                }
            }
        }
    }
}
