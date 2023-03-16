package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.StateConstants;
import frc.robot.subsystems.MotionControlSystem;
import frc.robot.subsystems.Arm.Claw;

public class ConeIntake extends CommandBase{
    private final MotionControlSystem m_motionSystem;
    private final Claw m_claw;
    private boolean m_finished = false;

    
    public ConeIntake(MotionControlSystem motionSystem, Claw claw){
        m_motionSystem = motionSystem;
        m_claw = claw;
    }

    @Override
    public void initialize(){
        m_finished = false;
        SmartDashboard.putBoolean("ConeIntake", true);
        m_motionSystem.setState(StateConstants.kConeIntake);
        m_motionSystem.runCone(0.85,false);
    }

    @Override
    public void end(boolean interrupted){
        SmartDashboard.putBoolean("ConeIntake", false);
        m_motionSystem.coneIn();
        m_claw.setSpeed(-750);
    }
    
    public void forceCancel(){
        m_finished = true;
    }

    @Override
    public boolean isFinished(){
        return m_finished;
    }
}