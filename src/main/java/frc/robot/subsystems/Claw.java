package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Claw extends SubsystemBase{
    CANSparkMax mMotor1 = new CANSparkMax(18, MotorType.kBrushless);
    CANSparkMax mMotor2 = new CANSparkMax(19, MotorType.kBrushless);
    RelativeEncoder mEncoder = mMotor1.getEncoder();
    SparkMaxPIDController mPID = mMotor1.getPIDController();

    public Claw(){
        mMotor1.setSmartCurrentLimit(CurrentLimit.kClaw);
        mMotor2.setSmartCurrentLimit(CurrentLimit.kClaw);
        mMotor1.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        mMotor2.enableVoltageCompensation(GlobalConstants.kVoltCompensation);

        mMotor1.setInverted(false);

        mMotor2.follow(mMotor1,true);

        mPID.setP(0.0001);
        mPID.setFF(0.00018);

        mPID.setSmartMotionMaxAccel(25000, 0);
        mPID.setSmartMotionMaxVelocity(5500, 0);

        mMotor1.burnFlash();
        mMotor2.burnFlash();

    }

    public void setSpeed(double speed){
        mPID.setReference(speed, ControlType.kSmartVelocity);
    }

    public void stop(){
        mMotor1.stopMotor();
        mMotor2.stopMotor();
    }

    @Override
    public void periodic(){

    }



}