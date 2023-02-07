package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class Odometry extends SubsystemBase {
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Drivetrain m_drive;
    private final Limelight m_vision;
    
    public Odometry(Drivetrain drive, Limelight vision) {
        m_drive = drive;
        m_vision = vision;

        m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, m_drive.getGyro(), m_drive.getModulePosititons(), new Pose2d());
    }

    @Override
    public void periodic() {
        //if (|visionPose - currentestimatedpose < 1 meter|) {
            m_poseEstimator.addVisionMeasurement(m_vision.getPose2d(), Timer.getFPGATimestamp());
        //}
        m_poseEstimator.update(m_drive.getGyro(), m_drive.getModulePosititons());
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        m_drive.resetOdometry(pose);
        m_poseEstimator.resetPosition(m_drive.getGyro(), m_drive.getModulePosititons(), pose);
    }

}
