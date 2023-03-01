package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class Dock extends CommandBase {
    private final Drivetrain m_drive;
    private final PIDController m_climbPID;
    private final PIDController m_levelingPID;

    private boolean m_climbing = true; // is the robot climbing or leveling
    private double m_levelEpoch = 0.0;

    private boolean m_finished = false;

    public Dock(Drivetrain drive) {
        m_drive = drive;
        m_climbPID = new PIDController(0.4, 0.0, 0.0);
        m_climbPID.setSetpoint(13.5);
        m_climbPID.setTolerance(1.5);
        m_levelingPID = new PIDController(0.05, 0.0, 0.0);
        m_levelingPID.setSetpoint(0.0);
        m_levelingPID.setTolerance(2.0);

        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        if (m_climbPID.atSetpoint()) {
            m_climbing = false;
        }
        if (m_climbing) {
            m_drive.drive(m_climbPID.calculate(0.0), 0.0, 0.0, true, true);
        } else {
            m_drive.drive(m_levelingPID.calculate(0.0), 0.0, 0.0, true, true);
            if (m_levelingPID.atSetpoint() && m_levelEpoch == 0.0) {
                m_levelEpoch = Timer.getFPGATimestamp();
            }
            if (m_levelingPID.atSetpoint() && Timer.getFPGATimestamp() - m_levelEpoch >= 1.5) {
                m_drive.setModuleStates(DriveConstants.kLockedWheels);
                m_finished = true;
            }
            if (!m_levelingPID.atSetpoint()) {
                m_levelEpoch = 0.0;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_climbing = true;
        m_levelEpoch = 0.0;
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }

}
