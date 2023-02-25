package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class PoseEstimator extends SubsystemBase {
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Drivetrain m_drive;
    private final LimelightBackup m_vision;

    public PoseEstimator(Drivetrain drive, LimelightBackup limelight, Pose2d intialPose) {
        m_drive = drive;
        m_vision = limelight;

        m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, m_drive.getGyro(), m_drive.getModulePositions(), intialPose);
    }

    @Override
    public void periodic() {
        updatePoseEstimator();
        updateShuffleboard();
    }

    private void updatePoseEstimator() {
        double timestamp = Timer.getFPGATimestamp() - (m_vision.getTotalLatency() / 1000.0);
        Pose2d currentPose = m_poseEstimator.getEstimatedPosition();
        Pose2d visionPose = getVisionPose();
        if (Math.abs(Math.sqrt( Math.pow(visionPose.getX(), 2) + Math.pow(visionPose.getY(), 2)) - Math.sqrt( Math.pow(currentPose.getX(), 2) + Math.pow(currentPose.getY(), 2))) <= VisionConstants.kPoseErrorAcceptance) {
            m_poseEstimator.addVisionMeasurement(visionPose, timestamp);
        }
        m_poseEstimator.updateWithTime(Timer.getFPGATimestamp() - (m_vision.getTotalLatency() / 1000.0), m_drive.getGyro(), m_drive.getModulePositions());
    }

    private void updateShuffleboard() {
        Pose2d pose = getPose();
        double[] poseArray = {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
        SmartDashboard.putNumberArray("Robot Pose", poseArray);
    }

    public Pose2d getPose() {
        Pose2d est = m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), m_drive.getGyro(), m_drive.getModulePositions());
        if (!FieldConstants.kAlliance) {
            return est;
        } else {
            return new Pose2d(est.getX(), FieldConstants.kFieldWidth - est.getY(), est.getRotation().times(-1));
        }
    }

    public Pose2d getPose(boolean allianceOrient) {
        Pose2d est = m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), m_drive.getGyro(), m_drive.getModulePositions());
        if (!allianceOrient || !FieldConstants.kAlliance) {
            return est;
        } else {
            return new Pose2d(est.getX(), FieldConstants.kFieldWidth - est.getY(), est.getRotation().times(-1));
        }
    }

    public boolean inside(Translation2d[] bounds, boolean onEdge) {
        Pose2d currentPose = getPose();
        double xMin = Math.min(bounds[0].getX(), bounds[1].getX());
        double xMax = Math.max(bounds[0].getX(), bounds[1].getX());
        double yMin = Math.min(bounds[0].getY(), bounds[1].getY());
        double yMax = Math.max(bounds[0].getY(), bounds[1].getY());
        return (
            (currentPose.getX() > xMin && currentPose.getX() < xMax) || (onEdge && (currentPose.getX() >= xMin && currentPose.getX() <= xMax)) 
            && 
            (currentPose.getY() > yMin && currentPose.getY() < yMax) || (onEdge && (currentPose.getY() >= yMin && currentPose.getY() <= yMax))
        );
    }

    private Pose2d getVisionPose() {
        return getPose();
    }

    public void resetOdometry(Pose2d pose) {
        m_drive.resetOdometry(pose);
        m_poseEstimator.resetPosition(m_drive.getGyro(), m_drive.getModulePositions(), pose);
    }

}
