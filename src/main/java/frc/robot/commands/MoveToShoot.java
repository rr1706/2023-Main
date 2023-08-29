package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimator;

public class MoveToShoot extends CommandBase {
    private final Drivetrain m_drivetrain;
    private final PoseEstimator m_poseEstimator;
    private final Limelight m_vision;
    
    public MoveToShoot(Drivetrain drivetrain, PoseEstimator poseEstimator, Limelight vision) {
        m_drivetrain = drivetrain;
        m_poseEstimator = poseEstimator;
        m_vision = vision;
    }

    @Override
    public void initialize() {
        
    }

}
