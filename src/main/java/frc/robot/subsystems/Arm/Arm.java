package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CurrentLimit;

public class Arm extends SubsystemBase {
    private final CANSparkMax m_motor1;
    private final CANSparkMax m_motor2;
    private final SparkMaxPIDController m_PID;

    private TrapezoidProfile.State state = new TrapezoidProfile.State(ArmConstants.kDefaultArm, 0.0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public Arm(int motorID1, int motorID2) {
        m_motor1 = new CANSparkMax(motorID1, MotorType.kBrushless);
        m_motor2 = new CANSparkMax(motorID2, MotorType.kBrushless);
        m_PID = m_motor1.getPIDController();

        m_motor2.follow(m_motor1, true);

        m_PID.setP(0.01);
        m_PID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        m_PID.setFeedbackDevice(m_motor1.getEncoder());

        m_motor1.setSoftLimit(SoftLimitDirection.kForward, (float) ArmConstants.kMinArm);
        m_motor1.setSoftLimit(SoftLimitDirection.kReverse, (float) ArmConstants.kMaxArm);
        m_motor1.setSmartCurrentLimit(CurrentLimit.kArm);
        m_motor1.enableVoltageCompensation(12.0);
        m_motor2.enableVoltageCompensation(12.0);
        m_motor1.burnFlash();
        m_motor2.burnFlash();
    }

    @Override
    public void periodic() {
        state = new TrapezoidProfile.State(getPose(), getVelocity());
        TrapezoidProfile profile = new TrapezoidProfile(ArmConstants.kArmConstraints, setpoint, state);
        state = profile.calculate(0.020);
        m_PID.setReference(setpoint.position,ControlType.kPosition,0,0.0);
    }

    public void resetEncoder() {
        m_motor1.getEncoder().setPosition(0.0);
    }

    public void resetEncoder(double pose) {
        m_motor1.getEncoder().setPosition(pose);
    }

    public double getPose() {
        return m_motor1.getEncoder().getPosition();
    }

    public void setPose(double pose) {
        setpoint = new TrapezoidProfile.State(pose, 0.0);
    }

    public double getVelocity() {
        return m_motor1.getEncoder().getVelocity();
    }
    
}
