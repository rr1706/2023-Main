package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.IntakeConstants;

public class Cone extends SubsystemBase {
    private final CANSparkMax m_motorExt;
    private final CANSparkMax m_motor;
    private final SparkMaxPIDController m_PID;

    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State state = new TrapezoidProfile.State();
 
    public Cone(int motorID1, int motorID2) {
        m_motorExt = new CANSparkMax(motorID1, MotorType.kBrushless);
        m_motor = new CANSparkMax(motorID2, MotorType.kBrushless);
        m_PID = m_motorExt.getPIDController();

        m_motorExt.setSoftLimit(SoftLimitDirection.kForward, 30);
        m_motorExt.setSoftLimit(SoftLimitDirection.kReverse, 0);
        m_PID.setP(0.01);
        m_motor.setSmartCurrentLimit(CurrentLimit.kCone);
        m_motorExt.setSmartCurrentLimit(CurrentLimit.kConeExt);
        m_motor.enableVoltageCompensation(12.0);
        m_motorExt.enableVoltageCompensation(12.0);
        m_motor.burnFlash();
        m_motorExt.burnFlash();

        state = new TrapezoidProfile.State(IntakeConstants.kRetractedCone, 0.0);
        setpoint = state;
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

    public void extend(double goal) {
        setpoint = new TrapezoidProfile.State(goal, 0.0);
        m_motor.set(0.500);
    }

    public void extend() {
        setpoint = new TrapezoidProfile.State(IntakeConstants.kExtendedCone, 0.0);
    }

    public void retract() {
        setpoint = new TrapezoidProfile.State(IntakeConstants.kRetractedCone, 0.0);
        m_motor.stopMotor();
    }

    @Override
    public void periodic() {
        state = new TrapezoidProfile.State(getPose(), getVelocity());
        TrapezoidProfile profile = new TrapezoidProfile(IntakeConstants.kConeConstraints, setpoint, state);
        state = profile.calculate(0.020);
        m_PID.setReference(setpoint.position, ControlType.kPosition, 0, 0.0);
    }

    public double getPose() {
        return m_motorExt.getEncoder().getPosition();
    }

    public double getVelocity() {
        return m_motorExt.getEncoder().getVelocity();
    }

}
