package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Dashboard {
    private final NetworkTableInstance m_instance;
    private final NetworkTable m_table;
    
    public Dashboard(String name) {
        m_instance = NetworkTableInstance.create();
        if (name == "") {
            m_table = m_instance.getTable("Dashboard");
        } else if (isFullName(name)) {
            m_table = m_instance.getTable(name);
        } else {
            m_table = m_instance.getTable(name + " Dashboard");
        }
    }

    /* 
     * Test whether the dashboard name contains the word "Dashboard" or some reasonable variation of it
    */
    private boolean isFullName(String name) {
        return name.matches("/[dD][aA][sS][hH]([bB]([oO]([aA]([rR]([dD])?)?)?)?)?/g");
    }
}
