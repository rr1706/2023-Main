package frc.robot.subsystems;

import org.json.simple.JSONArray;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private static  NetworkTable m_table;
    private static JSONArray jsonDump;
    private static ObjectMapper objectMapper = new ObjectMapper();

    public Limelight(String name, Pose3d pose) {
        m_table = NetworkTableInstance.getDefault().getTable(name);
    }

    @Override
    public void periodic() {
        jsonDump = objectMapper.writeValue(null, m_table.getEntry("json").getString("null"));
    }
    
    /**
     * 
     * @return True if there is a valid target for the current pipeline.
     */
    public boolean validTarget() {
        return m_table.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /**
     * 
     * @return The horizontal offset from the crosshair.
     */
    public double hOffset() {
        return m_table.getEntry("tx").getDouble(0.0);
    }

    /**
     * 
     * @return The vertical offset from the crosshair.
     */
    public double vOffset() {
        return m_table.getEntry("ty").getDouble(0.0);
    }

    /**
     * 
     * @return Total latency (photons -> robot) in milliseconds.
     */
    public double getLatency() {
        return m_table.getEntry("tl").getDouble(5.0) + 12.0;
    }

    /**
     * 
     * @return The ID of the primary april tag. Returns -1 if there is no april tag detected.
     */
    public int getFiducialID() {
        return (int) (m_table.getEntry("tid").getDouble(-1.0));
    }

    /**
     * 
     * @return Robot pose in 3D field space.
     */
    public Pose3d getPose3D() {
        double[] botpose = m_table.getEntry("botpose").getDoubleArray(new double[]{0.0});
        if (botpose == new double[]{0.0}) {
            return null;
        } else {
            Translation3d trans = new Translation3d(botpose[0], botpose[1], botpose[2]);
            Rotation3d rot = new Rotation3d(botpose[3], botpose[4], botpose[5]);
            return new Pose3d(trans, rot);
        }
    }

}
