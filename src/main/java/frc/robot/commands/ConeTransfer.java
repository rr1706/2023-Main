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
    private boolean m_timeReached1 = false;
    private boolean m_timeReached2 = false;
    private boolean m_finished = false;

    public ConeTransfer(MotionControlSystem motionSystem, Claw claw){
        m_motionSystem = motionSystem;
        m_claw = claw;
    }

    @Override
    public void initialize(){
        m_finished = false;
        m_timeReached1 = false;
        m_timeReached2 = false;
        m_timer.reset();
        m_timer.start();
        m_claw.setSpeed(-750);  
        m_motionSystem.runCone(-0.075,true); 

    }

    @Override
    public void execute(){
        if(m_timer.get() > 0.10 && !m_timeReached1){
            m_timeReached1 = true;
            m_motionSystem.runElevatorUp(11.0);
            m_motionSystem.runCone(-0.2,true); 
        }
        if(m_timer.get() > 0.4 && !m_timeReached2){
            m_timeReached2 = true;
            m_claw.setSpeed(-1000);

        }
        else if(m_timer.get() > 0.85){
            m_motionSystem.runCone(0.0,false);
            m_claw.stop();
            m_finished = true;
        }
    }

    @Override
    public void end(boolean interrupted){
        m_motionSystem.coneIn();
        m_motionSystem.runCone(0.0,false);
        m_claw.stop();    
    }

    @Override
    public boolean isFinished(){
        return m_finished;
    }
    
}
