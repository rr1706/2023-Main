package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.StateConstants;
import frc.robot.subsystems.MotionControlSystem;

public class Grab extends CommandBase {
    private final MotionControlSystem m_motionControl;
    private final boolean m_alliance;
    
    public Grab(MotionControlSystem motionControl, boolean useAllianceColor) {
        m_motionControl = motionControl;
        m_alliance = useAllianceColor;
    }

    @Override
    public void initialize() {
        if (m_alliance && DriverStation.getAlliance() == Alliance.Red) {
            m_motionControl.setState(StateConstants.kGrabRed);
        } else if (m_alliance && DriverStation.getAlliance() == Alliance.Blue) {
            m_motionControl.setState(StateConstants.kGrabBlue);
        } else {
            m_motionControl.setState(StateConstants.kGrab);
        }
        
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
