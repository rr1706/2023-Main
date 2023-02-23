package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;

public class LimelightBackup extends SubsystemBase {
    private final NetworkTable m_lime;
    private final HashMap<String, Double> lightStatus = new HashMap<>();
    private final String m_name;
    
    private boolean m_poleScoring = false;

    public LimelightBackup(String name) {
        lightStatus.put("Pipeline Controlled", 0.0);
        lightStatus.put("Off", 1.0);
        lightStatus.put("Blinking", 2.0);
        lightStatus.put("On", 3.0);
        m_lime = NetworkTableInstance.getDefault().getTable(name);
        m_lime.getEntry("ledMode").setDouble(lightStatus.get("Off"));
        m_name =  name;
    }

    public String getName() {
        return m_name;
    }

    /**
     * 
     * @return The robot's 2D pose according to the limelight
     */
    public Pose2d getPose2d() {
        double[] poseArray = m_lime.getEntry("botpose").getDoubleArray(new double[0]);
        return new Pose2d(
            new Translation2d(poseArray[0], poseArray[1]),
            new Rotation2d(poseArray[3], poseArray[4])
        );
    }
    
    /**
     * 
     * @return The robot's 3D pose according to the limelight
     */
    public Pose3d getPose3d() {
        double[] poseArray = m_lime.getEntry("botpose").getDoubleArray(new double[0]);
        return new Pose3d(
            new Translation3d(poseArray[0], poseArray[1], poseArray[2]),
            new Rotation3d(poseArray[3], poseArray[4], poseArray[5])
        );
    }

    /**
     * 
     * @return Total vision latency (photons -> robot) in milliseconds
     */
    public double getTotalLatency() {
        return m_lime.getEntry("tl_cap").getDouble(12.5);
    }

    /**
     * 
     * @return Get the pipeline index.
     */
    public int getPipeline() {
        return (int) m_lime.getEntry("getpipe").getDouble(0.0);
    }

    /**
     * Set the limelight pipeline index.
     * @param pipelineIndex
     */
    public void setPipeline(int pipelineIndex) {
        m_lime.getEntry("pipeline").setDouble((double) pipelineIndex);
    }

    /**
     * Set the limelight pipeline based on robot pose and/or intended action
     * @param poleScoring True if you intend to score on the poles
     */
    public void updateLimelightPipeline(boolean poleScoring, Pose2d pose) {
        m_poleScoring = poleScoring;
        /*
         * Limelight Pipeline Indexes
         * 0 - TagScoring
         * 1 - RetroScoring
         * 2 - ForeField
         * 3 - MidField
         * 4 - LoadingZone
         */
        // Scoring Zone
        if (pose.getX() > FieldConstants.kScoringZone[0] && pose.getX() < FieldConstants.kScoringZone[1] && pose.getY() > FieldConstants.kScoringZone[2] && pose.getY() < FieldConstants.kScoringZone[3] && (getPipeline() != 0 || getPipeline() != 1)) {
            if (m_poleScoring) {setPipeline(1);} else {setPipeline(0);}
        }
        // ForeField
        if (pose.getX() > FieldConstants.kForeField[0] && pose.getX() < FieldConstants.kForeField[1] && pose.getY() > FieldConstants.kForeField[2] && pose.getY() < FieldConstants.kForeField[3] && getPipeline() != 2) {
            setPipeline(2);
        }
        // MidField
        if (pose.getX() > FieldConstants.kMidField[0] && pose.getX() < FieldConstants.kMidField[1] && pose.getY() > FieldConstants.kMidField[2] && pose.getY() < FieldConstants.kMidField[3] && getPipeline() != 3) {
            setPipeline(3);
        }
        // Loading Zone
        if (pose.getX() > FieldConstants.kLoadingZone[0] && pose.getX() < FieldConstants.kLoadingZone[1] && pose.getY() > FieldConstants.kLoadingZone[2] && pose.getY() < FieldConstants.kLoadingZone[3] && getPipeline() != 4) {
            setPipeline(4);
        }
    }

    /**
     * Set the limelight pipeline based on robot pose and/or intended action
     * @param poleScoring True if you intend to score on the poles
     */
    public void updateLimelightPipeline(Pose2d pose) {
        /*
         * Limelight Pipeline Indexes
         * 0 - TagScoring
         * 1 - RetroScoring
         * 2 - ForeField
         * 3 - MidField
         * 4 - LoadingZone
         */
        // Scoring Zone
        if (pose.getX() > FieldConstants.kScoringZone[0] && pose.getX() < FieldConstants.kScoringZone[1] && pose.getY() > FieldConstants.kScoringZone[2] && pose.getY() < FieldConstants.kScoringZone[3] && (getPipeline() != 0 || getPipeline() != 1)) {
            if (m_poleScoring) {setPipeline(1);} else {setPipeline(0);}
        }
        // ForeField
        if (pose.getX() > FieldConstants.kForeField[0] && pose.getX() < FieldConstants.kForeField[1] && pose.getY() > FieldConstants.kForeField[2] && pose.getY() < FieldConstants.kForeField[3] && getPipeline() != 2) {
            setPipeline(2);
        }
        // MidField
        if (pose.getX() > FieldConstants.kMidField[0] && pose.getX() < FieldConstants.kMidField[1] && pose.getY() > FieldConstants.kMidField[2] && pose.getY() < FieldConstants.kMidField[3] && getPipeline() != 3) {
            setPipeline(3);
        }
        // Loading Zone
        if (pose.getX() > FieldConstants.kLoadingZone[0] && pose.getX() < FieldConstants.kLoadingZone[1] && pose.getY() > FieldConstants.kLoadingZone[2] && pose.getY() < FieldConstants.kLoadingZone[3] && getPipeline() != 4) {
            setPipeline(4);
        }
    }

    /**
     * Set the limelight crop window to increase frame rate.
     * @param borders [X-Min, X-Max, Y-Min, Y-Max]
     */
    public void setCropWindow(double[] borders) {
        m_lime.getEntry("crop").setDoubleArray(borders);
    }

    /**
     * 
     * @param status Pipeline Controller, Off, Blinking, On
     */
    public void setLights(String status) {
        m_lime.getEntry("ledMode").setDouble(lightStatus.get(status));
    }

}
