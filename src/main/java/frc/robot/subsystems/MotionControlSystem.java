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
import frc.robot.subsystems.Intake.Cube;
import frc.robot.subsystems.Arm.Elevator;
import frc.robot.subsystems.Arm.Wrist;

public final class MotionControlSystem extends SubsystemBase {
    private final Arm mArm = new Arm(16, 17);
    private final Cube mCube = new Cube(10, 11);
    private final Elevator mElevator = new Elevator(12, 13);
    private final Wrist mWrist = new Wrist(15);

    private MotionControlState mDesiredState = StateConstants.kHome;
    private MotionControlState mSetState = StateConstants.kHome;
    private boolean elevatorClearForArm = false;

    public MotionControlSystem(){}

    public void setState(MotionControlState desiredState){
        mDesiredState = desiredState;
    }

    @Override
    public void periodic(){
        //MotionControlState currentState = new MotionControlState(mArm.getPose(),mCube.getPose(),mElevator.getPose(),mWrist.getPose());
         
        SmartDashboard.putNumber("Elevator Desired", mDesiredState.m_elevator);
        SmartDashboard.putNumber("Arm Desired", mDesiredState.m_arm);
        SmartDashboard.putNumber("Wrist Desired", mDesiredState.m_wrist);

        SmartDashboard.putNumber("Current Elevator", mElevator.getPose());
        SmartDashboard.putNumber("Current Arm", mArm.getPose());
        SmartDashboard.putNumber("Current Wrist", mWrist.getPose());


        mArm.setPose(mDesiredState.m_arm);
        mElevator.setPose(mDesiredState.m_elevator);
        mWrist.setPose(mDesiredState.m_wrist);
        

        /* //Arm Movement Safety Logic
        boolean armMoveNeedsHeight //Does the arm movement require the elevator to move up or not move down yet?
            = ((currentState.m_arm < ArmConstants.kArmRev) && (mDesiredState.m_arm > ArmConstants.kArmRev)) ||
            ((currentState.m_arm > ArmConstants.kArmFwd) && (mDesiredState.m_arm < ArmConstants.kArmFwd));

        boolean armHeightMoveComplete
            = ((currentState.m_arm < ArmConstants.kArmRev) && (mDesiredState.m_arm < ArmConstants.kArmRev)) ||
            ((currentState.m_arm > ArmConstants.kArmFwd) && (mDesiredState.m_arm > ArmConstants.kArmFwd));

       if((armMoveNeedsHeight) && (currentState.m_elevator < ElevatorConstants.kClearHeightMin || mDesiredState.m_elevator < ElevatorConstants.kClearHeightMin) && !elevatorClearForArm){
           if(mDesiredState.m_elevator > ElevatorConstants.kClearHeight){
               mSetState.setElevator(mDesiredState.m_elevator);
               mElevator.setPose(mDesiredState.m_elevator);
               elevatorClearForArm = true;
           }
           else{
               mSetState.setElevator(ElevatorConstants.kClearHeight);
               mElevator.setPose(ElevatorConstants.kClearHeight);
               elevatorClearForArm = true;
           }

        }
        else if(armMoveNeedsHeight && currentState.m_elevator >= ElevatorConstants.kClearHeightMin || !armMoveNeedsHeight){
           mSetState.setArm(mDesiredState.m_arm);
           mArm.setPose(mDesiredState.m_arm);
           mWrist.setPose(mDesiredState.m_wrist);
        }        
        //Wrist Movement Safety Logic
        boolean wristNeedsHeight = (mDesiredState.m_wrist > WristConstants.kWristForwardMin); */
    }
}
