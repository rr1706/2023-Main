package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.StateConstants;
import frc.robot.subsystems.MotionControlSystem;
import frc.robot.subsystems.Arm.Claw;

public class ConeTransfer extends CommandBase{
    private final MotionControlSystem m_motionSystem;
    private final Claw m_claw;
    private final Timer m_timer = new Timer();
    private double m_time = 0.0;
    private boolean m_metSetpointOnce = false;
    private boolean m_finished = false;

    public ConeTransfer(MotionControlSystem motionSystem, Claw claw){
        m_motionSystem = motionSystem;
        m_claw = claw;
        
        addRequirements(m_motionSystem, m_claw);
    }

    @Override
    public void initialize(){
        m_finished = false;
        m_metSetpointOnce = false;
        m_timer.reset();
        m_timer.start();
        m_claw.setSpeed(-2000);  
        m_time = 0;
    }

    @Override
    public void execute(){
        if(!m_metSetpointOnce && m_motionSystem.atSetpoint() && m_timer.get() > 0.25){            
            m_time = m_timer.get();
            m_motionSystem.runCone(-0.4, true);
            m_metSetpointOnce = true;
        }
        if(m_timer.get()-m_time > 0.4 && m_metSetpointOnce){
            m_claw.setSpeed(-250);
            m_finished = true;
        }
    }

    @Override
    public void end(boolean interrupted){
        m_motionSystem.runCone(0.0,false);
        m_motionSystem.setState(StateConstants.kStart);
    }

    @Override
    public boolean isFinished(){
        return m_finished;
    }
    
}
