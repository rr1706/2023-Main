package frc.robot.subsystems;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;

public class PoseEstimator extends SubsystemBase {
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Drivetrain m_drive;
    private final Limelight m_vision;
    
    public PoseEstimator(Drivetrain drive, Limelight vision) {
        m_drive = drive;
        m_vision = vision;

        m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, m_drive.getGyro(), m_drive.getModulePosititons(), new Pose2d());
    }

    @Override
    public void periodic() {
        Pose2d currentPose = getPose(false);
        Pose2d visionPose = m_vision.getPose2d();
        double timestamp = Timer.getFPGATimestamp();
        if (Math.abs(Math.sqrt( Math.pow(visionPose.getX(), 2) + Math.pow(visionPose.getY(), 2)) - Math.sqrt( Math.pow(currentPose.getX(), 2) + Math.pow(currentPose.getY(), 2))) <= 1.0) {
            m_poseEstimator.addVisionMeasurement(visionPose, timestamp);
        }
        m_poseEstimator.update(m_drive.getGyro(), m_drive.getModulePosititons());
        updateShuffleboard();
    }

    public void updateShuffleboard() {
        Pose2d pose = getPose();
        double[] poseArray = {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
        SmartDashboard.putNumberArray("Robot Pose", poseArray);
    }

    public Pose2d getPose() {
        Pose2d est = m_poseEstimator.getEstimatedPosition();
        if (OperatorConstants.kAlliance) {
            return est;
        } else {
            return new Pose2d(1.654 - est.getX(), est.getY(), est.getRotation());
        }
    }

    public Pose2d getPose(boolean allianceOrient) {
        Pose2d est = m_poseEstimator.getEstimatedPosition();
        if (OperatorConstants.kAlliance || !allianceOrient) {
            return est;
        } else {
            return new Pose2d(1.654 - est.getX(), est.getY(), est.getRotation());
        }
    }

    public void resetOdometry(Pose2d pose) {
        m_drive.resetOdometry(pose);
        m_poseEstimator.resetPosition(m_drive.getGyro(), m_drive.getModulePosititons(), pose);
    }

}
