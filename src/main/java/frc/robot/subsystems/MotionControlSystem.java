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

    private MotionControlState m_desiredState = new MotionControlState(StateConstants.kHome) ;
    private boolean m_elevatorClear = false;
    private boolean atSetpoint = false;

    public MotionControlSystem(){
    }

    public void setState(MotionControlState desiredState){

        SmartDashboard.putNumber("ElevatorState from Command", desiredState.m_elevator);
        SmartDashboard.putNumber("ElevatorState from Constants", StateConstants.kConeIntake.m_elevator);

        m_desiredState.setState(desiredState);

        m_elevatorClear = false;
    }

    @Override
    public void periodic(){
        MotionControlState currentState = new MotionControlState(m_arm.getPose(),m_cube.getPose(),m_elevator.getPose(),m_wrist.getPose(),m_cone.getPose());

        if(!m_elevatorClear){
            if(m_desiredState.m_elevator < -9.0){
                m_elevator.setPose(-9.0);
            }
            else{
                m_elevator.setPose(m_desiredState.m_elevator);
            }
        }
        if(currentState.m_elevator >= -10.0 && !m_elevatorClear){
            m_elevatorClear = true;
            m_cube.setPose(m_desiredState.m_cube);
            m_arm.setPose(m_desiredState.m_arm);
            m_wrist.setPose(m_desiredState.m_wrist);
            m_cone.setPose(m_desiredState.m_cone);
        }

        atSetpoint = m_arm.atSetpoint() && m_cube.atSetpoint() && m_cone.atSetpoint() && m_elevator.atSetpoint() && m_wrist.atSetpoint();

        if(m_elevatorClear && atSetpoint){
            m_elevator.setPose(m_desiredState.m_elevator);
        }

        SmartDashboard.putNumber("Elevator Desired", m_desiredState.m_elevator);
        SmartDashboard.putNumber("Arm Desired", m_desiredState.m_arm);
        SmartDashboard.putNumber("Wrist Desired", m_desiredState.m_wrist);
        SmartDashboard.putNumber("Cone Desired", m_desiredState.m_cone);
        SmartDashboard.putNumber("Cube Desired", m_desiredState.m_cube);


        SmartDashboard.putNumber("Current Elevator", m_elevator.getPose());
        SmartDashboard.putNumber("Current Arm", m_arm.getPose());
        SmartDashboard.putNumber("Current Wrist", m_wrist.getPose());
        SmartDashboard.putNumber("Current Cone", m_cone.getPose());
        SmartDashboard.putNumber("Current Cube", m_cube.getPose());

    }

    public boolean isFinished() {
        return atSetpoint;
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

    public void runCone(double speed){
        m_cone.set(speed);
    }

    public void runElevatorUp(double pose){
        m_desiredState.setElevator(m_desiredState.m_elevator+pose);
    }

}
