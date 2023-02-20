package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.IntakeConstants;

public class Cube extends SubsystemBase {
    private final CANSparkMax m_motorExt;
    private final CANSparkMax m_motor;
    private final SparkMaxPIDController m_PID;
    private final RelativeEncoder m_extEncoder;
    private final RelativeEncoder m_encoder;

    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_state = new TrapezoidProfile.State();
 
    public Cube() {
        m_motor = new CANSparkMax(IntakeConstants.kCubeMotors[0], MotorType.kBrushless);
        m_motorExt = new CANSparkMax(IntakeConstants.kCubeMotors[1], MotorType.kBrushless);
        m_PID = m_motorExt.getPIDController();
        m_extEncoder = m_motorExt.getEncoder();
        m_encoder = m_motor.getEncoder();
        m_motorExt.setSoftLimit(SoftLimitDirection.kForward, (float) IntakeConstants.kExtendedCube);
        m_motorExt.setSoftLimit(SoftLimitDirection.kReverse, (float) IntakeConstants.kRetractedCube);
        m_PID.setP(IntakeConstants.kCubeP);
        m_PID.setFF(IntakeConstants.kCubeFF);
        m_motor.setSmartCurrentLimit(CurrentLimit.kCube);
        m_motorExt.setSmartCurrentLimit(CurrentLimit.kCubeExt);
        m_motor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motorExt.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor.burnFlash();
        m_motorExt.burnFlash();

        m_state = new TrapezoidProfile.State(IntakeConstants.kRetractedCube, 0.0);
        m_setpoint = m_state;
    }

    public void set(double speed) {
        m_motor.set(speed);
    }

    public void resetEncoder() {
        m_motorExt.getEncoder().setPosition(0.0);
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public void setPose(double pose) {
        m_setpoint = new TrapezoidProfile.State(pose, 0.0);
    }

    public void extend() {
        m_setpoint = new TrapezoidProfile.State(IntakeConstants.kExtendedCube, 0.0);
    }

    public void retract() {
        m_setpoint = new TrapezoidProfile.State(IntakeConstants.kRetractedCube, 0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Cube Setpoint", m_setpoint.position);
        m_PID.setReference(m_setpoint.position, ControlType.kSmartMotion, 0, 0.0);
    }

    public double getPose() {
        return m_extEncoder.getPosition();
    }

    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    public boolean atSetpoint() {
        return Math.abs(m_setpoint.position-m_extEncoder.getPosition()) <= 1.0;
    }

}
