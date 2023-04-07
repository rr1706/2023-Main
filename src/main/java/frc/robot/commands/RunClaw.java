package frc.robot.commands;

import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Arm.Claw;

public class RunClaw extends CommandBase{
    private final GenericHID m_operatorBoard;
    private final Claw m_claw;
    private final Limelight m_limelight;
    private final InterpolatingTreeMap<Double,Double> m_table = new InterpolatingTreeMap<>();

    private boolean useLockedPosition = false;
    private final int lockedPosition;

    public RunClaw(GenericHID operatorBoard, Limelight limelight, Claw claw){
        m_operatorBoard = operatorBoard;
        m_claw = claw;
        m_limelight = limelight;
        lockedPosition = -1;

        m_table.put(52.0, 2750.0);
        m_table.put(56.5, 2950.0);
        m_table.put(63.0, 3500.0);


    }
    public RunClaw(GenericHID operatorBoard, Limelight limelight, Claw claw, int scorePosition){
        m_operatorBoard = operatorBoard;
        m_claw = claw;
        lockedPosition = scorePosition;
        m_limelight = limelight;
        useLockedPosition = true;
    }


    @Override
    public void initialize(){
        boolean coneMid = false;
        boolean coneHigh = false;
        boolean cubeMid = false;
        boolean cubeHigh = false;
        boolean low = false;
        if(useLockedPosition){
            switch(lockedPosition){
                case 0: break;
                case 1: low = true;
                case 2: coneMid = true; break;
                case 3: coneHigh = true; break;
                case 4: cubeMid = true; break;
                case 5: cubeHigh = true; break;
                default: break;
            }
        }
        else{
            low = m_operatorBoard.getRawButton(4) || m_operatorBoard.getRawButton(5) || m_operatorBoard.getRawButton(6);
            coneMid = m_operatorBoard.getRawButton(7) || m_operatorBoard.getRawButton(9);
            coneHigh = m_operatorBoard.getRawButton(10) || m_operatorBoard.getRawButton(12);
            cubeMid = m_operatorBoard.getRawButton(8);
            cubeHigh =  m_operatorBoard.getRawButton(11); 
        }
    
        if(cubeMid){
            m_claw.setSpeed(1500);
        }
        else if(cubeHigh){
            m_claw.setSpeed(2000);
        }
        else if(coneMid){
            m_claw.setSpeed(1100);
        }
        else if(coneHigh){
            m_claw.setSpeed(m_table.get(m_limelight.getDist()));
        }
        else if(low){
            m_claw.setSpeed(1000);
        }
        else{
            m_claw.setSpeed(4500);
        }
    }

    @Override
    public void end(boolean interrupted){
        m_claw.stop();
    }

}