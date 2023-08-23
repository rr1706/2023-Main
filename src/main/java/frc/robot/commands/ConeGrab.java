package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm.Claw;

public class ConeGrab extends CommandBase {
    private final Claw m_claw;
    private final Drivetrain m_drivetrain;

    public ConeGrab(Claw claw, Drivetrain drivetrain){
        m_claw = claw;
        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speed = m_drivetrain.getChassisSpeed().vxMetersPerSecond;
        double output = 2000.0 + 1500.0*speed;
        if(output<=2000.0) {output=2000.0;}
        m_claw.setSpeed(-output);

    }

    @Override
    public void end(boolean interrupted) {
        m_claw.setSpeed(-50);
    }
    
}
