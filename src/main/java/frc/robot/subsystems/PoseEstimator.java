package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class PoseEstimator extends SubsystemBase {
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Drivetrain m_drive;
    private final Limelight m_vision;

    public PoseEstimator(Drivetrain drive, Limelight limelight, Pose2d intialPose) {
        m_drive = drive;
        m_vision = limelight;

        m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, m_drive.getGyro(), m_drive.getModulePosititons(), intialPose);
    }

    @Override
    public void periodic() {
        updatePoseEstimator();
        updateShuffleboard();
        m_vision.updateLimelightPipeline(getPose());
    }

    private void updatePoseEstimator() {
        double timestamp = Timer.getFPGATimestamp() - (m_vision.getTotalLatency() / 1000.0);
        Pose2d currentPose = m_poseEstimator.getEstimatedPosition();
        Pose2d visionPose = getVisionPose();
        if (Math.abs(Math.sqrt( Math.pow(visionPose.getX(), 2) + Math.pow(visionPose.getY(), 2)) - Math.sqrt( Math.pow(currentPose.getX(), 2) + Math.pow(currentPose.getY(), 2))) <= VisionConstants.kPoseErrorAcceptance) {
            m_poseEstimator.addVisionMeasurement(visionPose, timestamp);
        }
        m_poseEstimator.updateWithTime(Timer.getFPGATimestamp() - (m_vision.getTotalLatency() / 1000.0), m_drive.getGyro(), m_drive.getModulePosititons());
    }

    private void updateShuffleboard() {
        Pose2d pose = getPose();
        double[] poseArray = {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
        SmartDashboard.putNumberArray("Robot Pose", poseArray);
    }

    public Pose2d getPose() {
        Pose2d est = m_poseEstimator.updateWithTime(Timer.getFPGATimestamp() - (m_vision.getTotalLatency() / 1000.0), m_drive.getGyro(), m_drive.getModulePosititons());
        if (!FieldConstants.kAlliance) {
            return est;
        } else {
            return new Pose2d(est.getX(), FieldConstants.kFieldWidth - est.getY(), est.getRotation().times(-1));
        }
    }

    public Pose2d getPose(boolean allianceOrient) {
        Pose2d est = m_poseEstimator.updateWithTime(Timer.getFPGATimestamp() - (m_vision.getTotalLatency() / 1000.0), m_drive.getGyro(), m_drive.getModulePosititons());
        if (!allianceOrient || !FieldConstants.kAlliance) {
            return est;
        } else {
            return new Pose2d(est.getX(), FieldConstants.kFieldWidth - est.getY(), est.getRotation().times(-1));
        }
    }

    private Pose2d getVisionPose() {
        return getPose().plus(new Transform2d(VisionConstants.kLimelightToRobot.toPose2d().getTranslation(), VisionConstants.kLimelightToRobot.toPose2d().getRotation()));
    }

    public void resetOdometry(Pose2d pose) {
        m_drive.resetOdometry(pose);
        m_poseEstimator.resetPosition(m_drive.getGyro(), m_drive.getModulePosititons(), pose);
    }

}
