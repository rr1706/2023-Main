package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmsConstants;
import frc.robot.Constants.CurrentLimit;

public class Wrist extends SubsystemBase {
    private final CANSparkMax m_motor;
    private final SparkMaxPIDController m_PID;
    private final RelativeEncoder m_encoder;

    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    public Wrist() {
        m_motor = new CANSparkMax(ArmsConstants.kWristMotor, MotorType.kBrushless);
        m_PID = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();

        m_PID.setP(0.00001);
        m_PID.setFF(0.00009);

        m_PID.setSmartMotionMaxAccel(12500, 0);
        m_PID.setSmartMotionMaxVelocity(4000, 0);

        m_motor.setSoftLimit(SoftLimitDirection.kForward, (float) ArmsConstants.kMaxWrist);
        m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float) ArmsConstants.kMinWrist);
        m_motor.setSmartCurrentLimit(CurrentLimit.kWrist);
        m_motor.enableVoltageCompensation(12.0);
        m_motor.burnFlash();

        m_setpoint = new TrapezoidProfile.State(getPose(),0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Setpoint", m_setpoint.position);
        m_PID.setReference(m_setpoint.position, ControlType.kSmartMotion,0,0.0);
    }

    public void resetEncoder() {
        m_encoder.setPosition(0.0);
    }

    public void resetEncoder(double pose) {
        m_encoder.setPosition(pose);
    }

    public double getPose() {
        return  m_encoder.getPosition();
    }

    public void setPose(double pose) {
        m_setpoint = new TrapezoidProfile.State(pose, 0.0);
    }

    public double getVelocity() {
        return m_encoder.getVelocity();
    
    }

    public boolean atSetpoint() {
        return Math.abs(m_setpoint.position-m_encoder.getPosition()) <= 4.0;
    }
    
}
