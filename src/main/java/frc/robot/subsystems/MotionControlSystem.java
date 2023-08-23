package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmsConstants;
import frc.robot.Constants.StateConstants;
import frc.robot.utilities.MotionControlState;

import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Intake.Cone;
import frc.robot.subsystems.Intake.Cube;
import frc.robot.subsystems.Arm.Elevator;
import frc.robot.subsystems.Arm.Wrist;

public final class MotionControlSystem extends SubsystemBase {
    private final Arm m_arm = new Arm();
    private final Cube m_cube = new Cube();
    private final Cone m_cone = new Cone();
    private final Elevator m_elevator = new Elevator();
    private final Wrist m_wrist = new Wrist();
    private double m_offset = 0.0;

    private MotionControlState m_desiredState;// = new MotionControlState(StateConstants.kHome) ;
    private boolean m_elevatorClear = false;
    private boolean m_forceCube = false;
    private boolean m_runCube = false;

    public MotionControlSystem() {
        SmartDashboard.putBoolean("+0.5 to ARM", false);
        SmartDashboard.putBoolean("-0.5 to ARM", false);
        m_desiredState = getMotionState();
    }

    public void setState(MotionControlState desiredState){

       // SmartDashboard.putNumber("ElevatorState from Command", desiredState.m_elevator);
        //SmartDashboard.putNumber("ElevatorState from Constants", StateConstants.kConeIntake.m_elevator);

        m_desiredState.setState(desiredState);

        m_elevatorClear = false;
    }

    @Override
    public void periodic(){
        MotionControlState currentState = new MotionControlState(m_arm.getPose(),m_cube.getPose(),m_elevator.getPose(),m_wrist.getPose(),m_cone.getPose());

        if(m_forceCube){
            m_desiredState.setCube(22.0);
        }

        if(SmartDashboard.getBoolean("+0.5 to ARM", false)){
            m_offset += 0.5;
            SmartDashboard.putBoolean("+0.5 to ARM", false);
            setState(m_desiredState);
        }
        else if(SmartDashboard.getBoolean("-0.5 to ARM", false)){
            m_offset -= 0.5;
            SmartDashboard.putBoolean("-0.5 to ARM", false);
            setState(m_desiredState);
        }
        

        double clearHeight = -9.0;
        if(m_desiredState.m_cube > 20.0 && m_cube.getPose() <= 20.0 || m_desiredState.m_cube < 5.0 && m_cube.getPose() >= 5.0 || m_forceCube){
            clearHeight = -2.0;
        }

        boolean armAlreadySafe = m_desiredState.m_arm > 20.0 && currentState.m_arm > 20.0; 

        if(!m_elevatorClear){
            if(m_desiredState.m_elevator < clearHeight){
                m_elevator.setPose(clearHeight);
            }
            else{
                m_elevator.setPose(m_desiredState.m_elevator);
            }
        }

        if((currentState.m_elevator >= clearHeight-1.0 && !m_elevatorClear) || (armAlreadySafe && !m_elevatorClear)){
            m_elevatorClear = true;
            m_arm.setPose(m_desiredState.m_arm+m_offset);
            m_wrist.setPose(m_desiredState.m_wrist);
            m_cone.setPose(m_desiredState.m_cone);
            m_cube.setPose(m_desiredState.m_cube);
        }

        boolean atSetpoint = m_arm.atSetpoint() && m_cube.atSetpoint() && m_cone.atSetpoint() && m_elevator.atSetpoint() && m_wrist.atSetpoint();

        if((m_elevatorClear && atSetpoint) || armAlreadySafe){
            m_elevator.setPose(m_desiredState.m_elevator);
        }

        if((cubeAtSetpoint() && m_runCube)){
            m_cube.set(-0.45);
        }
        else if(!m_forceCube){
            m_cube.set(0.0);
        }

        SmartDashboard.putNumber("Elevator Desired", m_desiredState.m_elevator);

        SmartDashboard.putNumber("Arm Desired", m_desiredState.m_arm+m_offset);

        SmartDashboard.putNumber("Wrist Desired", m_desiredState.m_wrist);
        SmartDashboard.putNumber("Cone Desired", m_desiredState.m_cone);
        SmartDashboard.putNumber("Cube Desired", m_desiredState.m_cube);


        SmartDashboard.putNumber("Current Elevator", m_elevator.getPose());
        SmartDashboard.putNumber("Current Arm", m_arm.getPose());
        SmartDashboard.putNumber("Current Wrist", m_wrist.getPose());
        SmartDashboard.putNumber("Current Cone", m_cone.getPose());
        SmartDashboard.putNumber("Current Cube", m_cube.getPose());

        SmartDashboard.putBoolean("MotionReady", atSetpoint());

    }

    public boolean atSetpoint(){
        return m_arm.atSetpoint() && m_cube.atSetpoint() && m_cone.atSetpoint() && m_elevator.atSetpoint() && m_wrist.atSetpoint();
    }

    public boolean cubeAtSetpoint(){
        return m_cube.atSetpoint();
    }

    public boolean clearForHigh(){
        return m_elevator.getPose() >= -5.0;
    }

    public boolean clearForMid(){
        return m_arm.getPose() >= 35.0;
    }


    public void coneIn(){
        m_cone.setPose(0.0);
        m_cone.set(0.0);
    }

    public void runCone(double speed, boolean force){
        m_cone.setForce(force);
        if(force){
            m_cone.set(speed);
        }
        else{
            m_cone.set(speed);
        }
    }

    public void runCubeWhenReady(boolean run){
        m_runCube = run;
    }

    public void runCone(double speed){
        m_cone.set(speed);
    }

    public void runCube(double speed){
        m_cube.set(speed);
    }
    

    public void runElevatorUp(double pose){
        m_desiredState.setElevator(m_desiredState.m_elevator+pose);
    }

    public void toggleCube(){
       if(m_forceCube){
         m_desiredState.setCube(0.0);
       }
       m_forceCube = !m_forceCube;
       m_elevatorClear = false;
    }
    public void forceCubeIn(){
        m_desiredState.setCube(3.0);
        m_forceCube = false;
        m_elevatorClear = false;
     }
     public MotionControlState getMotionState(){
        return new MotionControlState(m_arm.getPose(), m_cube.getPose(), m_elevator.getPose(), m_wrist.getPose(), m_cone.getPose());
     }

}