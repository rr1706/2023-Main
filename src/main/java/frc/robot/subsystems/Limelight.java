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

import frc.robot.Constants.VisionConstants;

public class Limelight extends SubsystemBase {
    private final NetworkTable m_lime;
    private final HashMap<String, Double> lightStatus = new HashMap<>();

    public Limelight(String name) {
        lightStatus.put("Pipeline Controlled", 0.0);
        lightStatus.put("Off", 1.0);
        lightStatus.put("Blinking", 2.0);
        lightStatus.put("On", 3.0);
        m_lime = NetworkTableInstance.getDefault().getTable(name);
        m_lime.getEntry("ledMode").setDouble(1.0);
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
     * @return Total vision latency (photons -> robot).
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
