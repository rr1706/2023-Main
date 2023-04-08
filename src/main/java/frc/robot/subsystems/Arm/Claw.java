package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Claw extends SubsystemBase{
    CANSparkMax mMotor1 = new CANSparkMax(18, MotorType.kBrushless);
    CANSparkMax mMotor2 = new CANSparkMax(19, MotorType.kBrushless);
    RelativeEncoder mEncoder = mMotor1.getEncoder();
    SparkMaxPIDController mPID = mMotor1.getPIDController();

    SimpleMotorFeedforward mFF = new SimpleMotorFeedforward(0.015, 0.00016667);

    public Claw(){
        mMotor1.setSmartCurrentLimit(CurrentLimit.kClaw);
        mMotor2.setSmartCurrentLimit(CurrentLimit.kClaw);
        mMotor1.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        mMotor2.enableVoltageCompensation(GlobalConstants.kVoltCompensation);

        mMotor1.setInverted(false);

        mMotor2.follow(mMotor1,true);

        mEncoder.setAverageDepth(4);
        mEncoder.setMeasurementPeriod(16);

        mPID.setP(0.0001);
        mPID.setFF(0.0000);

        mPID.setSmartMotionMaxAccel(27500, 0);
        mPID.setSmartMotionMaxVelocity(5000, 0);

        mMotor1.burnFlash();
        mMotor2.burnFlash();

    }

    public void setSpeed(double speed){
        double ff = mFF.calculate(speed);
        mPID.setReference(speed, ControlType.kSmartVelocity,0,ff*GlobalConstants.kVoltCompensation);
    }

    public void stop(){
        mMotor1.stopMotor();
        mMotor2.stopMotor();
    }

    @Override
    public void periodic(){

        SmartDashboard.putNumber("Claw Speed", mEncoder.getVelocity());
    }



}
