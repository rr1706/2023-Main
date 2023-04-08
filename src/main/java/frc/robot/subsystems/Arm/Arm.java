package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
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
    private final AnalogPotentiometer m_absEncoder;
    private final ProfiledPIDController m_rioPID = new ProfiledPIDController(0.015,0.1
    ,0.00, new Constraints(450, 350));
    private final ArmFeedforward m_ff = new ArmFeedforward(0.00,0.045,0.0045);

    private boolean m_useABSEnc = true;

    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    public Arm() {
        m_rioPID.setIntegratorRange(-0.015, 0.015);
        m_motor1 = new CANSparkMax(ArmsConstants.kArmMotors[0], MotorType.kBrushless);
        m_motor2 = new CANSparkMax(ArmsConstants.kArmMotors[1], MotorType.kBrushless);
        m_PID = m_motor1.getPIDController();
        m_encoder = m_motor1.getEncoder();

        m_absEncoder = new AnalogPotentiometer(ArmsConstants.kArmAbsEncoder, 126.166,-40.88);

        m_motor2.follow(m_motor1, true);

        m_PID.setP(0.00001);
        m_PID.setFF(0.00009);

        m_PID.setSmartMotionMaxAccel(20000, 0);
        m_PID.setSmartMotionMaxVelocity(10000, 0);
        m_PID.setSmartMotionAllowedClosedLoopError(0.0, 0);

        m_motor1.setSoftLimit(SoftLimitDirection.kForward, (float) ArmsConstants.kMaxArm);
        m_motor1.setSoftLimit(SoftLimitDirection.kReverse, (float) ArmsConstants.kMinArm);
        m_motor2.enableSoftLimit(SoftLimitDirection.kForward, false);
        m_motor2.enableSoftLimit(SoftLimitDirection.kReverse, false);
        m_motor1.setSmartCurrentLimit(CurrentLimit.kArm);

        m_motor1.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor2.enableVoltageCompensation(GlobalConstants.kVoltCompensation);

        m_motor1.burnFlash();
        m_motor2.burnFlash();

        SmartDashboard.putBoolean("USE ANALOG ENC", true);

        m_setpoint = new TrapezoidProfile.State(getPose(),0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Setpoint", m_setpoint.position);
        SmartDashboard.putNumber("Arm ABS ENC", m_absEncoder.get());
        double output = m_rioPID.calculate(getPose(), m_setpoint);
        TrapezoidProfile.State state = m_rioPID.getSetpoint();
        SmartDashboard.putNumber("Arm Desired Velocity", state.velocity);
        SmartDashboard.putNumber("Arm Actual Velocity", getVelocity()/60.0);
        double ff = m_ff.calculate((state.position*Math.PI/90.0)-Math.PI/2.0, state.velocity);
        m_motor1.set(output+ff);

        m_useABSEnc = SmartDashboard.getBoolean("USE ANALOG ENC", true);
    }

    public void resetEncoder() {
        m_encoder.setPosition(0.0);
    }

    public void resetEncoder(double pose) {
        m_encoder.setPosition(pose);
    }

    public double getPose() {
        if(m_useABSEnc){
            return m_absEncoder.get();
        }else{
            return m_encoder.getPosition()-2.0;
        }
    }

    public void setPose(double pose) {
        m_setpoint = new TrapezoidProfile.State(pose, 0.0);
        m_rioPID.reset(getPose());
    }

    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    public boolean atSetpoint() {
        return Math.abs(m_setpoint.position-getPose()) <= 5.0;
    }
    
}