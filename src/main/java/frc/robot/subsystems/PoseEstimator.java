package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.utilities.MathUtils;

public class PoseEstimator extends SubsystemBase {
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Drivetrain m_drive;
    private final Limelight m_vision;
    private final Field2d m_field = new Field2d();
    public PoseEstimator(Drivetrain drive, Limelight limelight, Pose2d intialPose) {
        m_drive = drive;
        m_vision = limelight;
        SmartDashboard.putData("Field", m_field);

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
        m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), m_drive.getGyro(), m_drive.getModulePositions());
        if (Math.abs(MathUtils.pythagorean(currentPose.getX(), currentPose.getY()) - MathUtils.pythagorean(visionPose.getX(), visionPose.getY())) <= VisionConstants.kPoseErrorAcceptance) {
            m_poseEstimator.addVisionMeasurement(visionPose, timestamp);
        }
    }

    private void updateShuffleboard() {
        Pose2d pose = getPose();
        double[] poseArray = {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
        SmartDashboard.putNumberArray("Robot Pose", poseArray);
        SmartDashboard.putNumber("Robot X", poseArray[0]);
        SmartDashboard.putNumber("Robot Y",  poseArray[1]);
        SmartDashboard.putNumber("Robot Gyro",  poseArray[2]);
        m_field.setRobotPose(pose);   
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

    private Pose2d getVisionPose() {
        return m_vision.getPose3d().toPose2d();
    }

    public void resetOdometry(Pose2d pose) {
        m_drive.resetOdometry(pose);
        m_poseEstimator.resetPosition(m_drive.getGyro().times(-1.0), m_drive.getModulePositions(), pose);
    }

}
