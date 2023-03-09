package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class Dock extends CommandBase {
    private final Drivetrain m_drive;
    private final PIDController m_climbPID;
    private final PIDController m_levelingPID;
    private final double m_initialTilt;

    private boolean m_climbing = true; // is the robot climbing or leveling
    private boolean m_initialHump = false;
    private double m_levelEpoch = 0.0;
    private int directionFactor = -1;

    private boolean m_finished = false;

    public Dock(Drivetrain drive, boolean direction) {
        m_drive = drive;
        m_climbPID = new PIDController(0.15, 0.0, 0.0);
        m_climbPID.setSetpoint(-13.5*directionFactor);
        m_climbPID.setTolerance(1.5);
        m_levelingPID = new PIDController(0.04, 0.0, 0.0);
        m_levelingPID.setSetpoint(0.0);
        m_levelingPID.setTolerance(8.0);
        m_initialTilt = 0.0;
        if (direction) {
            directionFactor *= -1;
        }
            addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_climbing = true;
        m_levelEpoch = 0.0;
        m_climbPID.setSetpoint(-13.5*directionFactor);
        m_finished = false;
        m_initialHump = false;
        m_drive.drive(directionFactor*1.0, 0.0, 0.0, true, false);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("climbing", m_climbing);
        if (Math.abs(Math.abs(m_drive.getTilt()) - 13.5) <= 1.0) {
            m_climbing = false;
        }
        if (m_climbing) {
            m_climbPID.calculate(m_drive.getTilt());
            m_drive.drive(directionFactor * 1.0, 0.0, 0.0, true, false);
        } else {
            m_climbPID.calculate(m_drive.getTilt());
            if (((Math.abs(m_drive.getTiltVel()) >= 6.0)) && !m_finished) {
                m_levelingPID.calculate(m_drive.getTilt());
                if (!m_finished) {
                    if (!m_initialHump) {
                        m_levelEpoch = Timer.getFPGATimestamp();
                    }
                    m_drive.drive(directionFactor*  0.85, 0, 0, true, false);
                    m_initialHump = true;
                    if (Timer.getFPGATimestamp() - m_levelEpoch >= 1.6) {
                        m_finished = true;
                        m_drive.setModuleStates(DriveConstants.kLockedWheels);
                    }
                }
            }
            else if(m_finished){
                m_levelingPID.calculate(m_drive.getTilt());
                m_drive.setModuleStates(DriveConstants.kLockedWheels);
            } else {
                if (!m_finished) {

                    m_levelingPID.calculate(m_drive.getTilt());
                    m_drive.drive(directionFactor * 0.35, 0.0, 0.0, true, false);
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_climbing = true;
        m_levelEpoch = 0.0;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
