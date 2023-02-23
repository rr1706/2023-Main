package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.MotionControlSystem;

public class Score extends CommandBase {
    private final Drivetrain m_drivetrain;
    private final PoseEstimator m_poseEstimator;
    private final Limelight m_vision;
    private final MotionControlSystem m_motionControl;
    private final PIDCommand m_PID;

    public Score(Drivetrain drivetrain, PoseEstimator poseEstimator, Limelight vision, MotionControlSystem motionControl) {
        m_drivetrain = drivetrain;
        m_poseEstimator = poseEstimator;
        m_vision = vision;
        m_motionControl = motionControl;
        m_PID = new PIDCommand(new PIDController(0, 0, 0),
        m_drivetrain::getTilt,
        0.0,
        this::moveToScore,
        m_drivetrain, m_vision);
    }

    @Override
    public void execute() {

    }

    private void moveToScore(double pidOutput) {
        m_drivetrain.drive(0.0, pidOutput, 0.0, true, true);
    }

}
