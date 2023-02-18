// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.StateConstants;
import frc.robot.commands.DriveByController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MotionControlSystem;
import frc.robot.subsystems.PoseEstimator;

import java.io.File;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);

  private final Drivetrain m_drive = new Drivetrain();
  private final Limelight m_vision = new Limelight("limelight");
  private final PoseEstimator m_poseEstimator = new PoseEstimator(m_drive, m_vision, new Pose2d());
  private final MotionControlSystem m_motionControl = new MotionControlSystem();

  private final DriveByController m_driveByController = new DriveByController(m_drive, m_driverController);

  private SendableChooser<Command> m_chooser = new SendableChooser<>();
  private File[] m_autoPathFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/").listFiles();

  private final HashMap<String, Command> events = new HashMap<>();
  private final Command doNothin = new WaitCommand(20.0);
  private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(m_poseEstimator::getPose, m_poseEstimator::resetOdometry, new PIDConstants(0.25, 0, 0), new PIDConstants(ModuleConstants.kTurnPID[0], ModuleConstants.kTurnPID[1], ModuleConstants.kTurnPID[2]), m_drive::setModuleStates, events, true, m_drive);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureAutoEvents();
    configureAutoChooser();
    configureBindings();

    m_drive.setDefaultCommand(m_driveByController);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new POVButton(m_driverController, 0)
      .onTrue(new InstantCommand(() -> m_drive.resetOdometry(new Pose2d())));

    new JoystickButton(m_driverController, Button.kA.value).onTrue(new InstantCommand(() -> m_motionControl.setState(StateConstants.kHome)));
  }

  private void configureAutoEvents() {}

  private void configureAutoChooser() {
    m_chooser.setDefaultOption("Do Nothin", doNothin);
    

    for (File auto : m_autoPathFiles) {
      m_chooser.addOption(
        auto.getName(), 
        autoBuilder.fullAuto(PathPlanner.loadPathGroup(auto.getName().replace(".path", ""), DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAcceleration))
      );
    }

    for (File auto : m_autoPathFiles) {
      m_chooser.addOption(
        "Slow " + auto.getName(), 
        autoBuilder.fullAuto(PathPlanner.loadPathGroup(auto.getName().replace(".path", ""), DriveConstants.kTestMaxSpeedMetersPerSecond, DriveConstants.kTestMaxAcceleration))
      );
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }
}
