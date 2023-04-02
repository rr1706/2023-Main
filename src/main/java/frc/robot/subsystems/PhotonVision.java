package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator m_poseEstimator;

    public PhotonVision(String name) {
        m_camera = new PhotonCamera(name);
        try {
            m_poseEstimator = new PhotonPoseEstimator(AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField(), PoseStrategy.MULTI_TAG_PNP, m_camera, new Transform3d());
            m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            m_poseEstimator = null;
        }
    }

}
