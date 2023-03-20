package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private final NetworkTable m_lime;
    private final String m_name;
    private final ArrayList<double[]> m_poses = new ArrayList<>();

    private Alliance m_alliance = Alliance.Invalid;

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
        storePose(getPoseWithTimestamp());
        SmartDashboard.putNumber("Vision X", m_poses.get(m_poses.size() - 1)[0]);
        SmartDashboard.putNumber("Vision Y", m_poses.get(m_poses.size() - 1)[1]);
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

    public double getAltTY() {
        return Math.min(m_lime.getEntry("cy0").getDouble(999999), m_lime.getEntry("cy1").getDouble(999999));
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
    private double[] getPoseWithTimestamp() {
        double[] pose;
        double timestamp = Timer.getFPGATimestamp() - getTotalLatency() / 1000;
        double[] poseWithTimestamp = new double[7];
        if (m_alliance == Alliance.Blue) {
            pose = m_lime.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        } else if (m_alliance == Alliance.Red) {
            pose = m_lime.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        } else {
            pose = m_lime.getEntry("botpose").getDoubleArray(new double[6]);
        }
        for (int i = 0; i < 6; i++) {
            poseWithTimestamp[i] = pose[i];
        }
        if (poseWithTimestamp != new double[7]) {
            poseWithTimestamp[6] = timestamp;
        }
        return poseWithTimestamp;
    }

    private void storePose(double[] pose) {
        if (pose != new double[7]) {
            m_poses.add(pose);
        }
    }

    public double[] getLatestPose3d() {
        return m_poses.size() == 0 ? new double[7] : m_poses.remove(0);
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
        m_lime.getEntry("ledMode").setNumber(status);
    }

}
