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

        addRequirements(m_motionSystem);
    }

    @Override
    public void initialize(){
        m_finished = false;
        m_motionSystem.setState(StateConstants.kConeIntake);
        m_motionSystem.runCone(0.40,false);
    }

    @Override
    public void end(boolean interrupted){
        m_claw.setSpeed(-750);
        m_motionSystem.runCone(0.0,false);
        m_motionSystem.setState(StateConstants.kConeIntakeIn);
    }
    
    public void forceCancel(){
        m_finished = true;
    }

    @Override
    public boolean isFinished(){
        return m_finished;
    }
}
