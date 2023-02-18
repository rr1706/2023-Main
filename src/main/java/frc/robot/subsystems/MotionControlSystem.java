package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmsConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.StateConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.utilities.MotionControlState;

import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Intake.Cube;
import frc.robot.subsystems.Arm.Elevator;
import frc.robot.subsystems.Arm.Wrist;

public final class MotionControlSystem extends SubsystemBase {
    private final Arm mArm = new Arm(0, 0);
    private final Cube mCube = new Cube(0, 0);
    private final Elevator mElevator = new Elevator(0, 0);
    private final Wrist mWrist = new Wrist(0);

    private MotionControlState mDesiredState = StateConstants.kHome;
    private MotionControlState mSetState = StateConstants.kHome;
    private boolean elevatorClearForArm = false;

    public MotionControlSystem(){}

    public void setState(MotionControlState desiredState){
        mDesiredState = desiredState;
    }

    @Override
    public void periodic(){
        MotionControlState currentState = new MotionControlState(mArm.getPose(),mCube.getPose(),mElevator.getPose(),mWrist.getPose());
         
        //Arm Movement Safety Logic
        boolean armMoveNeedsHeight //Does the arm movement require the elevator to move up or not move down yet?
            = ((currentState.m_arm < ArmConstants.kArmRev) && (mDesiredState.m_arm > ArmConstants.kArmRev)) ||
            ((currentState.m_arm > ArmConstants.kArmFwd) && (mDesiredState.m_arm < ArmConstants.kArmFwd));

        boolean armHeightMoveComplete
            = ((currentState.m_arm < ArmConstants.kArmRev) && (mDesiredState.m_arm < ArmConstants.kArmRev)) ||
            ((currentState.m_arm > ArmConstants.kArmFwd) && (mDesiredState.m_arm > ArmConstants.kArmFwd));

       if((armMoveNeedsHeight) && (currentState.m_elevator < ElevatorConstants.kClearHeightMin || mDesiredState.m_elevator < ElevatorConstants.kClearHeightMin) && !elevatorClearForArm){
           if(mDesiredState.m_elevator > ElevatorConstants.kClearHeight){
               mSetState.setElevator(mDesiredState.m_elevator);
               elevatorClearForArm = true;
           }
           else{
               mSetState.setElevator(ElevatorConstants.kClearHeight);
               elevatorClearForArm = true;
           }

        }
        else if(armMoveNeedsHeight && currentState.m_elevator >= ElevatorConstants.kClearHeightMin){
           mSetState.m_arm = mDesiredState.m_arm;
        }        
        //Wrist Movement Safety Logic
        boolean wristNeedsHeight = (mDesiredState.m_wrist > WristConstants.kWristForwardMin);
    }
}
