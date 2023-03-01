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
import frc.robot.Constants.GlobalConstants;

public class Arm extends SubsystemBase {
    private final CANSparkMax m_motor1;
    private final CANSparkMax m_motor2;
    private final SparkMaxPIDController m_PID;
    private final RelativeEncoder m_encoder;

    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    public Arm() {
        m_motor1 = new CANSparkMax(ArmsConstants.kArmMotors[0], MotorType.kBrushless);
        m_motor2 = new CANSparkMax(ArmsConstants.kArmMotors[1], MotorType.kBrushless);
        m_PID = m_motor1.getPIDController();
        m_encoder = m_motor1.getEncoder();

        m_motor2.follow(m_motor1, true);

        m_PID.setP(0.00001);
        m_PID.setFF(0.00009);

        m_PID.setSmartMotionMaxAccel(20000, 0);
        m_PID.setSmartMotionMaxVelocity(10000, 0);

        m_motor1.setSoftLimit(SoftLimitDirection.kForward, (float) ArmsConstants.kMaxArm);
        m_motor1.setSoftLimit(SoftLimitDirection.kReverse, (float) ArmsConstants.kMinArm);
        m_motor2.enableSoftLimit(SoftLimitDirection.kForward, false);
        m_motor2.enableSoftLimit(SoftLimitDirection.kReverse, false);
        m_motor1.setSmartCurrentLimit(CurrentLimit.kArm);

        m_motor1.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor2.enableVoltageCompensation(GlobalConstants.kVoltCompensation);

        m_motor1.burnFlash();
        m_motor2.burnFlash();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Setpoint", m_setpoint.position);
        m_PID.setReference(m_setpoint.position,ControlType.kSmartMotion,0,0.0);
    }

    public void resetEncoder() {
        m_encoder.setPosition(0.0);
    }

    public void resetEncoder(double pose) {
        m_encoder.setPosition(pose);
    }

    public double getPose() {
        return m_encoder.getPosition();
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