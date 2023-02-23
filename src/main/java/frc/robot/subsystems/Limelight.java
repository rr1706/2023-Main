package frc.robot.subsystems;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;

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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;

public class Limelight extends SubsystemBase {
    private final NetworkTable m_lime;
    private final HashMap<String, Double> lightStatus = new HashMap<>();
    private final File m_limelightJson;
    private final String m_name;

    private Alliance m_alliance = Alliance.Invalid;
    private boolean m_poleScoring = false;

    public Limelight(String name) {
        m_lime = NetworkTableInstance.getDefault().getTable(name);
        m_lime.getEntry("ledMode").setDouble(1.0);
        m_name =  name;
        m_limelightJson = new File(Filesystem.getDeployDirectory() + "/" + m_name + ".json");
        try {
            if (m_limelightJson.createNewFile()) {
                System.out.println("Limelight json file created at " + m_limelightJson.getAbsolutePath());
            }
        } catch (IOException e) {
            System.out.println("Error creating limelight json file.");
            e.printStackTrace();
        }
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

    public void writeToJson() throws IOException {
        try {
            FileWriter jsonWriter = new FileWriter(m_limelightJson);
            jsonWriter.write(getJson());
            jsonWriter.close();
          } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
          }
    }

    /**
     * 
     * @return The robot's 2D pose according to the limelight
     */
    public Pose2d getPose2d() {
        if (m_alliance == Alliance.Blue) {
            double[] poseArray = m_lime.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
            return new Pose2d(
                new Translation2d(poseArray[0], poseArray[1]),
                new Rotation2d(poseArray[3], poseArray[4])
            );
        } else if (m_alliance == Alliance.Red) {
            double[] poseArray = m_lime.getEntry("botpose_wpired").getDoubleArray(new double[0]);
            return new Pose2d(
                new Translation2d(poseArray[0], poseArray[1]),
                new Rotation2d(poseArray[3], poseArray[4])
            );
        }
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
        if (m_alliance == Alliance.Blue) {
            double[] poseArray = m_lime.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
            return new Pose3d(
                new Translation3d(poseArray[0], poseArray[1], poseArray[2]),
                new Rotation3d(poseArray[3], poseArray[4], poseArray[5])
            );
        } else if (m_alliance == Alliance.Red) {
            double[] poseArray = m_lime.getEntry("botpose_wpired").getDoubleArray(new double[0]);
            return new Pose3d(
                new Translation3d(poseArray[0], poseArray[1], poseArray[2]),
                new Rotation3d(poseArray[3], poseArray[4], poseArray[5])
            );
        } else {
            double[] poseArray = m_lime.getEntry("botpose").getDoubleArray(new double[0]);
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
    public void updateLimelightPipeline(Pose2d pose, boolean poleScoring) {
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
     * @param status 0 = Pipeline Controlled, 1 = Off, 2 = On, 3 = Blinking
     */
    public void setLights(int status) {
        m_lime.getEntry("ledMode").setDouble((double) status);
    }

}
