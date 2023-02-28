package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FieldConstants;

public class Limelight extends SubsystemBase {
    private final NetworkTable m_lime;
    private final String m_name;

    private Alliance m_alliance = Alliance.Invalid;
    private boolean m_poleScoring = false;

    public Limelight(String name) {
        m_lime = NetworkTableInstance.getDefault().getTable(name);
        m_lime.getEntry("ledMode").setDouble(1.0);
        m_name =  name;
    }

    @Override
    public void periodic() {
        if (m_alliance == Alliance.Invalid) {
            m_alliance = DriverStation.getAlliance();
        }
    }

    public String getName() {
        return m_name;
    }

    public String getJson() {
        return m_lime.getEntry("json").getString("nothing");
    }

    public double getTX() {
        return m_lime.getEntry("tx").getDouble(999999);
    }

    public double getTY() {
        return m_lime.getEntry("ty").getDouble(999999);
    }

    /**
     * Singifies if the limelight currently has an accetable target, defaulting to
     * false if no value is provided in the Network Table.
     * This default is important so that if the limelight becomes disconnected or
     * gives bad numbers the code will assume there is not a valid target
     *
     * @return true if an acceptable target is visible.
     */
    public boolean valid() {
        return m_lime.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /**
     * Allows retreival of the target area.
     *
     * @return the area of the target.
     */
    public double getTA() {
        return m_lime.getEntry("ta").getDouble(0.0);
    }
    
    /**
     * 
     * @return The robot's 3D pose according to the limelight
     */
    public Pose3d getPose3d() {
        if (m_alliance == Alliance.Blue) {
            double[] poseArray = m_lime.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            return new Pose3d(
                new Translation3d(poseArray[0], poseArray[1], poseArray[2]),
                new Rotation3d(poseArray[3], poseArray[4], poseArray[5])
            );
        } else if (m_alliance == Alliance.Red) {
            double[] poseArray = m_lime.getEntry("botpose_wpired").getDoubleArray(new double[6]);
            return new Pose3d(
                new Translation3d(poseArray[0], poseArray[1], poseArray[2]),
                new Rotation3d(poseArray[3], poseArray[4], poseArray[5])
            );
        } else {
            double[] poseArray = m_lime.getEntry("botpose").getDoubleArray(new double[6]);
            return new Pose3d(
                new Translation3d(poseArray[0], poseArray[1], poseArray[2]),
                new Rotation3d(poseArray[3], poseArray[4], poseArray[5])
            );
        }
    }

    /**
     * 
     * @return Total vision latency (photons -> robot) in milliseconds
     */
    public double getTotalLatency() {
        return m_lime.getEntry("tl").getDouble(12.5) + m_lime.getEntry("cl").getDouble(0.0);
    }

    /**
     * 
     * @return Get the pipeline index.
     */
    public int getPipeline() {
        return (int) m_lime.getEntry("getpipe").getDouble(-1.0);
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
     * @param status 0 = Pipeline Controlled, 1 = Off, 2 = On, 3 = Blinking
     */
    public void setLights(int status) {
        m_lime.getEntry("ledMode").setDouble((double) status);
    }

}
