package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class Dock extends CommandBase {
    private final Drivetrain m_drive;

    private boolean m_climbing = true; // is the robot climbing or leveling
    private boolean m_initialHump = false;
    private double m_levelEpoch = 0.0;
    private int directionFactor;

    // Change these values to adjust docking
    private final double climbSpeed = 1.0;
    private final double levelSpeed = 0.5;
    private final double angleTolerance = 1.0;
    private final double angularSensitivity = 7.0;
    private final double bufferTime = 1.8;
    private final boolean keepAngle = false;

    private boolean m_finished = false;

    public Dock(Drivetrain drive, boolean direction) {
        m_drive = drive;
        directionFactor = direction ? -1 : 1;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_climbing = true;
        m_levelEpoch = 0.0;
        m_finished = false;
        m_initialHump = false;
        m_drive.drive(directionFactor*1.0, 0.0, 0.0, true, false);
    }

    @Override
    public void execute() {
        if (Math.abs( Math.abs(m_drive.getTilt()) - 13.5) <= angleTolerance) {
            m_climbing = false;
        }
        if (m_climbing) {
            m_drive.drive(directionFactor * climbSpeed, 0.0, 0.0, true, keepAngle);
        } else {
            m_climbPID.calculate(m_drive.getTilt());
            if (((Math.abs(m_drive.getTiltVel()) >= 12.0)) && !m_finished) {
                m_levelingPID.calculate(m_drive.getTilt());
                if (!m_finished) {
                    if (!m_initialHump) {
                        m_levelEpoch = Timer.getFPGATimestamp();
                    }
                    m_drive.drive(directionFactor*  0.85, 0, 0, true, false);
                    m_initialHump = true;
                    if (Timer.getFPGATimestamp() - m_levelEpoch >= 1.7) {
                        m_finished = true;
                        m_drive.setModuleStates(DriveConstants.kLockedWheels);
                    }
                }
                m_drive.drive(directionFactor * climbSpeed, 0.0, 0.0, true, keepAngle);
                m_initialHump = true;
                if (Timer.getFPGATimestamp() - m_levelEpoch >= bufferTime) {
                    m_finished = true;
                    m_drive.setModuleStates(DriveConstants.kLockedWheels);
                }
            } else if (m_finished) {
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
