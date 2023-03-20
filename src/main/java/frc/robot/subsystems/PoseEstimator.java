package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

public class PoseEstimator extends SubsystemBase {
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Drivetrain m_drive;
    private final Limelight m_vision;
    private final Field2d m_field = new Field2d();
    public PoseEstimator(Drivetrain drive, Limelight limelight, Pose2d intialPose) {
        m_drive = drive;
        m_vision = limelight;
        //SmartDashboard.putData("Field", m_field);

        m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, m_drive.getGyro(), m_drive.getModulePositions(), intialPose, VecBuilder.fill(0.1, 0.1, 0.05), VecBuilder.fill(20, 20, 100));
    }

    @Override
    public void periodic() {
        updatePoseEstimator();
        updateShuffleboard();
    }

    private void updatePoseEstimator() {
        double[] visionMeasurement = m_vision.getLatestPose3d();
        double timestamp = visionMeasurement[6];
        Pose2d currentPose = getPose();
        Pose2d visionPose = new Pose2d(
            new Translation2d(visionMeasurement[0], visionMeasurement[1]),
            new Rotation2d(visionMeasurement[5])
        );
        m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), m_drive.getGyro(), m_drive.getModulePositions());
        if (currentPose.getTranslation().getDistance(visionPose.getTranslation()) <= VisionConstants.kPoseErrorAcceptance) {
            m_poseEstimator.addVisionMeasurement(visionPose, timestamp, VecBuilder.fill(currentPose.getX() / 3, currentPose.getX() / 3, 100.0));
        }
    }

    private void updateShuffleboard() {
        Pose2d pose = getPose();
        double[] poseArray = {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
        //SmartDashboard.putNumberArray("Robot Pose", poseArray);
        SmartDashboard.putNumber("Robot X", poseArray[0]);
        SmartDashboard.putNumber("Robot Y",  poseArray[1]);
        SmartDashboard.putNumber("Robot Gyro",  poseArray[2]);
        //m_field.setRobotPose(pose);
     }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public Pose2d getPose(boolean allianceOrient) {
        return m_poseEstimator.getEstimatedPosition();
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

    public void resetOdometry(Pose2d pose) {
        m_drive.resetOdometry(new Pose2d(pose.getTranslation(), getPose().getRotation()));
        m_poseEstimator.resetPosition(m_drive.getGyro().times(-1.0), m_drive.getModulePositions(), pose);
    }

}
