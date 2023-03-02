package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Claw;

public class RunClaw extends CommandBase{
    private final GenericHID m_operatorBoard;
    private final Claw m_claw;
    public RunClaw(GenericHID operatorBoard, Claw claw){
        m_operatorBoard = operatorBoard;
        m_claw = claw;
    }

    @Override
    public void initialize(){
        boolean coneMid = m_operatorBoard.getRawButton(7) || m_operatorBoard.getRawButton(9);
        boolean coneHigh = m_operatorBoard.getRawButton(10) || m_operatorBoard.getRawButton(12);
        boolean cube = m_operatorBoard.getRawButton(8) || m_operatorBoard.getRawButton(11); 
    
        if(cube){
            m_claw.setSpeed(1500);
        }
        else if(coneMid){
            m_claw.setSpeed(1000);
        }
        else if(coneHigh){
            m_claw.setSpeed(2850);
        }
        else{
            m_claw.setSpeed(500);
        }
    }

    @Override
    public void end(boolean interrupted){
        m_claw.stop();
    }

}