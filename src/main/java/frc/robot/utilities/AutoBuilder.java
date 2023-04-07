package frc.robot.utilities;

import com.pathplanner.lib.auto.PIDConstants;

import frc.robot.subsystems.Drivetrain;

public class AutoBuilder {
    private final Drivetrain m_drivetrain;

    public AutoBuilder(String name, Drivetrain drivetrain, PIDConstants translationConstants, PIDConstants rotationConstants) {
        m_drivetrain = drivetrain;
    }

}
