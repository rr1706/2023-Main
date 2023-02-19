package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmsConstants;
import frc.robot.Constants.CurrentLimit;

public class Wrist extends SubsystemBase {
    private final CANSparkMax m_motor;
    private final SparkMaxPIDController m_PID;

    private TrapezoidProfile.State state = new TrapezoidProfile.State(ArmsConstants.kDefaultWrist, 0.0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public Wrist(int motorID) {
        m_motor = new CANSparkMax(motorID, MotorType.kBrushless);
        m_PID = m_motor.getPIDController();

        m_PID.setP(0.00001);
        m_PID.setFF(0.00009);

        m_PID.setSmartMotionMaxAccel(10000, 0);
        m_PID.setSmartMotionMaxVelocity(3000, 0);

        m_motor.setSoftLimit(SoftLimitDirection.kForward, (float) ArmsConstants.kMaxWrist);
        m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float) ArmsConstants.kMinWrist);
        m_motor.setSmartCurrentLimit(CurrentLimit.kWrist);
        m_motor.enableVoltageCompensation(12.0);
        m_motor.burnFlash();
    }

    @Override
    public void periodic() {
        //state = new TrapezoidProfile.State(getPose(), getVelocity());
        //TrapezoidProfile profile = new TrapezoidProfile(ArmsConstants.kWristConstraints, setpoint, state);
        //state = profile.calculate(0.020);
        SmartDashboard.putNumber("Wrist Setpoint", setpoint.position);
        m_PID.setReference(setpoint.position,ControlType.kSmartMotion,0,0.0);
    }

    public void resetEncoder() {
        m_motor.getEncoder().setPosition(0.0);
    }

    public void resetEncoder(double pose) {
        m_motor.getEncoder().setPosition(pose);
    }

    public double getPose() {
        return m_motor.getEncoder().getPosition();
    }

    public void setPose(double pose) {
        setpoint = new TrapezoidProfile.State(pose, 0.0);
    }

    public double getVelocity() {
        return m_motor.getEncoder().getVelocity();
    }
    
}
