// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.StateConstants;
import frc.robot.commands.DriveByController;
import frc.robot.commands.Score;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MotionControlSystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.utilities.JoystickLeftTrigger;
import frc.robot.utilities.JoystickRightTrigger;
import frc.robot.utilities.OperatorBoard;

import java.io.File;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OperatorConstants.kOperatorControllerPort);

  /**
   * @see Button Values are 1-12, going from left -> right, bottom -> top
   */
  private final GenericHID m_operatorBoard = new GenericHID(OperatorConstants.kOperatorBoardPort);

  private final Drivetrain m_drive = new Drivetrain();
  private final Limelight m_vision = new Limelight("limelight");
  private PoseEstimator m_poseEstimator = new PoseEstimator(m_drive, m_vision, new Pose2d());
  private final MotionControlSystem m_motionControl = new MotionControlSystem();
  private final Claw m_claw = new Claw();

  private final DriveByController m_driveByController = new DriveByController(m_drive, m_driverController);

  private SendableChooser<Command> m_chooser = new SendableChooser<>();
  private File[] m_autoPathFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/").listFiles();

  private final HashMap<String, Command> events = new HashMap<>();
  private final Command doNothin = new WaitCommand(20.0);
  private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(m_poseEstimator::getPose, m_poseEstimator::resetOdometry, new PIDConstants(0.25, 0, 0), new PIDConstants(ModuleConstants.kTurnPID[0], ModuleConstants.kTurnPID[1], ModuleConstants.kTurnPID[2]), m_drive::setModuleStates, events, true, m_drive, m_vision, m_poseEstimator, m_claw);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureAutoEvents();
    configureAutoChooser();
    configureBindings();

    m_drive.setDefaultCommand(m_driveByController);
  }

  private void configureBindings() {
    new POVButton(m_driverController, 0)
      .onTrue(new InstantCommand(() -> m_drive.resetOdometry(new Pose2d())));
    new POVButton(m_driverController, 180)
      .onTrue(new InstantCommand(() -> m_drive.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)))));

    new JoystickButton(m_driverController, Button.kA.value).whileTrue(new Score(m_drive, m_poseEstimator, m_vision, m_motionControl, m_claw, OperatorBoard.selectedPosition(m_operatorBoard), OperatorBoard.selectedHeight(m_operatorBoard)));
    new JoystickButton(m_driverController, Button.kX.value).onTrue(new InstantCommand(() -> m_motionControl.setState(StateConstants.kGrab)));
    new JoystickButton(m_driverController, Button.kY.value).onTrue(new InstantCommand(() -> m_motionControl.setState(StateConstants.kShoot)));
    new JoystickButton(m_driverController, Button.kB.value).onTrue(new InstantCommand(() -> m_motionControl.setState(StateConstants.kFloor)));
    new JoystickButton(m_driverController, Button.kLeftBumper.value).onTrue(new InstantCommand(() -> m_motionControl.setState(StateConstants.kMidShoot)));

    new JoystickLeftTrigger(m_operatorController).onTrue(new InstantCommand(()-> m_motionControl.setState(StateConstants.kConeIntake)).alongWith(new InstantCommand(()->m_motionControl.runCone(0.5,false)))).onFalse(new InstantCommand(()->m_motionControl.coneIn()));

    new JoystickLeftTrigger(m_driverController).onTrue(new InstantCommand(()->m_claw.setSpeed(-500))).onFalse(new InstantCommand(()->m_claw.stop()));
    
    new JoystickRightTrigger(m_operatorController).onTrue(new WaitCommand(0.35).alongWith(new InstantCommand(()->m_claw.setSpeed(-250)).alongWith(new InstantCommand(()->m_motionControl.runCone(-0.35, true))).alongWith(new InstantCommand(()->m_motionControl.runElevatorUp(5.0)))).andThen((new InstantCommand(()->m_claw.setSpeed(-2000))).alongWith(new InstantCommand(()->m_motionControl.runCone(0.0,false))))).onFalse(new InstantCommand(()->m_motionControl.runCone(0.0,false)).alongWith(new InstantCommand(()->m_claw.stop())));

    new JoystickRightTrigger(m_driverController).onTrue(new InstantCommand(()->m_claw.setSpeed(3000))).onFalse(new InstantCommand(()->m_claw.stop()));
    new JoystickButton(m_driverController, Button.kRightBumper.value).onTrue(new InstantCommand(()->m_claw.setSpeed(1000))).onFalse(new InstantCommand(()->m_claw.stop()));
  }

  private void configureAutoEvents() {
    events.put("Low", new InstantCommand(() -> m_motionControl.setState(StateConstants.kLow)));
    events.put("ConeIntake", new InstantCommand(() -> m_motionControl.setState(StateConstants.kConeIntake)));
    events.put("Floor", new InstantCommand(() -> m_motionControl.setState(StateConstants.kFloor)));
    events.put("Grab", new InstantCommand(() -> m_motionControl.setState(StateConstants.kGrab)));
    events.put("ConeHigh", new InstantCommand(() -> m_motionControl.setState(StateConstants.kConeHigh)));
    events.put("ConeMid", new InstantCommand(() -> m_motionControl.setState(StateConstants.kConeMid)));
    events.put("Home", new InstantCommand(() -> m_motionControl.setState(StateConstants.kHome)));
    events.put("CubeHigh", new InstantCommand(() -> m_motionControl.setState(StateConstants.kCubeHigh)));
    events.put("CubeMid", new InstantCommand(() -> m_motionControl.setState(StateConstants.kCubeMid)));
  }

  private void configureAutoChooser() {
    m_chooser.setDefaultOption("Do Nothin", doNothin);

    for (File auto : m_autoPathFiles) {
      if (auto.getName().contains(".path")) {
        m_chooser.addOption(
          auto.getName(), 
          autoBuilder.fullAuto(PathPlanner.loadPathGroup(auto.getName().replace(".path", ""), DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAcceleration))
        );
      }
    }

    for (File auto : m_autoPathFiles) {
      if (auto.getName().contains(".path")) {
        m_chooser.addOption(
          "Slow " + auto.getName(), 
          autoBuilder.fullAuto(PathPlanner.loadPathGroup(auto.getName().replace(".path", ""), DriveConstants.kTestMaxSpeedMetersPerSecond, DriveConstants.kTestMaxAcceleration))
        );
      }
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
