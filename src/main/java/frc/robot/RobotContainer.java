// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.StateConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.ConeGrab;
import frc.robot.commands.ConeIntake;
import frc.robot.commands.ConeTransfer;
import frc.robot.commands.Dock;
import frc.robot.commands.DriveByController;
import frc.robot.commands.Grab;
import frc.robot.commands.RunClaw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MotionControlSystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.utilities.JoystickLeftTrigger;
import frc.robot.utilities.JoystickRightTrigger;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  private final XboxController m_operatorController = new XboxController(OperatorConstants.kOperatorControllerPort);

  /**
   * @see Button Values are 1-12, going from left -> right, bottom -> top
   */
  private final GenericHID m_operatorBoard = new GenericHID(OperatorConstants.kOperatorBoardPort);

  private final Drivetrain m_drive = new Drivetrain();
  private final Limelight m_vision = new Limelight("limelight-new");
  private final Limelight m_topVision = new Limelight("limelight-top");
  private final PoseEstimator m_poseEstimator = new PoseEstimator(m_drive, m_topVision, new Pose2d());
  private final MotionControlSystem m_motionControl = new MotionControlSystem();
  private final Claw m_claw = new Claw();

  private final AutoAlign m_align = new AutoAlign(m_drive, m_motionControl, m_claw, m_driverController, m_operatorBoard, m_topVision, m_vision);
  //private final RunClaw m_runClaw = new RunClaw(m_operatorBoard,m_vision, m_claw);
  private final ConeIntake m_coneIntake = new ConeIntake(m_motionControl, m_claw);
  private final ConeTransfer m_coneTransfer = new ConeTransfer(m_motionControl, m_claw);

  private final ConeIntake m_autoConeIntake = new ConeIntake(m_motionControl, m_claw);
  private final ConeTransfer m_autoConeTransfer = new ConeTransfer(m_motionControl, m_claw);

  private final DriveByController m_driveByController = new DriveByController(m_drive, m_driverController);

  private SendableChooser<Command> m_chooser = new SendableChooser<>();
  private File[] m_autoPathFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/").listFiles();

  private final HashMap<String, Command> events = new HashMap<>();
  private final Command doNothin = new WaitCommand(20.0);
  private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(m_poseEstimator::getPose, m_poseEstimator::resetOdometry, new PIDConstants(5.0, 0, 0), new PIDConstants(5.0,0.0,0), m_drive::setModuleStates, events, true, m_drive, m_vision, m_claw);
  private final SwerveAutoBuilder balanceBuilder = new SwerveAutoBuilder(m_drive::getPose, m_drive::resetOdometry, new PIDConstants(5.0, 0, 0), new PIDConstants(5.0,0.0,0), m_drive::setModuleStates, events, true, m_drive, m_vision, m_claw);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureAutoEvents();
    configureAutoChooser();
    configureBindings();
    m_vision.setPipeline(0);
    m_drive.setDefaultCommand(m_driveByController);
  }

  private void configureBindings() {
    new POVButton(m_driverController, 0)
    .onTrue(new InstantCommand(() -> m_drive.resetOdometry(new Pose2d())));
  new POVButton(m_driverController, 180)
    .onTrue(new InstantCommand(() -> m_drive.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)))));
  new POVButton(m_driverController, 90)
    .onTrue(new InstantCommand(() -> m_poseEstimator.resetOdometry(new Pose2d(new Translation2d(1.85,3.08),new Rotation2d(Math.PI)))));

  //new JoystickButton(m_driverController, Button.kY.value).onTrue(new InstantCommand(() -> m_motionControl.setState(StateConstants.kCoolThing))).onFalse(new InstantCommand(()->m_motionControl.setState(StateConstants.kHome)));

  new JoystickButton(m_driverController, Button.kStart.value).onTrue(new InstantCommand(() -> m_motionControl.setState(StateConstants.kStart)));
  new JoystickButton(m_driverController, Button.kY.value).onTrue(new InstantCommand(() -> m_motionControl.setState(StateConstants.kUnJam)));

  new JoystickButton(m_driverController, Button.kA.value).whileTrue(m_align).onFalse( new InstantCommand(()->m_drive.changeSlewRate(8.0,16.0)));
  new JoystickButton(m_driverController, Button.kX.value).onTrue(new Grab(m_motionControl, true)).onTrue(new InstantCommand(()->m_drive.changeSlewRate(5.0,12.0)));

  new JoystickButton(m_driverController, Button.kB.value).onTrue(new InstantCommand(()->m_motionControl.forceCubeIn()).andThen(new InstantCommand(() -> m_motionControl.setState(StateConstants.kFloor))));
  
  new JoystickButton(m_driverController, Button.kRightBumper.value).whileTrue(m_coneIntake).onFalse(m_coneTransfer);
  new JoystickButton(m_driverController, Button.kLeftBumper.value).onTrue(new InstantCommand(() -> m_motionControl.setState(StateConstants.kCube)).alongWith(new InstantCommand(()->m_motionControl.runCubeWhenReady(true))).alongWith(new InstantCommand(()->m_claw.setSpeed(-1250)))).onFalse(new InstantCommand(() -> m_motionControl.setState(StateConstants.kHome)).alongWith(new InstantCommand(()->m_motionControl.runCubeWhenReady(false))).alongWith(new InstantCommand(()->m_claw.setSpeed(-250))).alongWith(new InstantCommand(()->m_motionControl.forceCubeIn())));
  //new JoystickButton(m_driverController, Button.kY.value).onTrue(new InstantCommand(()->m_motionControl.toggleCube()).alongWith(new InstantCommand(()->m_motionControl.runCube(-0.15)))).onFalse(new InstantCommand(()->m_motionControl.runCube(0.0)));

  //new JoystickLeftTrigger(m_operatorController).onTrue(new InstantCommand(()-> m_motionControl.setState(StateConstants.kConeIntake)).alongWith(new InstantCommand(()->m_motionControl.runCone(0.5,false)))).onFalse(new InstantCommand(()->m_motionControl.coneIn()));

  //new JoystickLeftTrigger(m_driverController).onTrue(new InstantCommand(()->m_claw.setSpeed(-2500))).onFalse(new InstantCommand(()->m_claw.setSpeed(-50.0)));
  
  new JoystickLeftTrigger(m_driverController).whileTrue(new ConeGrab(m_claw, m_drive));

  new JoystickButton(m_operatorController, Button.kA.value).whileTrue(m_align);

  //new Trigger(()->m_operatorBoard.getRawButton(2)).onTrue(new InstantCommand(()->m_motionControl.setState(StateConstants.kSuper))).onFalse(new InstantCommand(()->m_motionControl.setState(StateConstants.kHome)));

  new JoystickRightTrigger(m_driverController).onTrue(new ConditionalCommand(new WaitCommand(1.0),new ConditionalCommand(new InstantCommand(()->m_claw.setSpeed(4500)),new InstantCommand(()->m_claw.setSpeed(1500)),()->m_operatorBoard.getRawButton(2)),()->m_align.isScheduled())).onFalse(new ConditionalCommand(new WaitCommand(1.0),new InstantCommand(()->m_claw.setSpeed(0.0)),()->m_align.isScheduled()));
  }

  private void configureAutoEvents() {
    events.put("Low", new InstantCommand(() -> m_motionControl.setState(StateConstants.kLow)));
    events.put("Floor", new InstantCommand(() -> m_motionControl.setState(StateConstants.kFloor)));
    events.put("Grab", new InstantCommand(() -> m_motionControl.setState(StateConstants.kGrab)));
    events.put("ConeHigh", new InstantCommand(() -> m_motionControl.setState(StateConstants.kConeHigh)));
    events.put("ConeMid", new InstantCommand(() -> m_motionControl.setState(StateConstants.kConeMid)));
    events.put("Home", new InstantCommand(() -> m_motionControl.setState(StateConstants.kHome)));
    events.put("CubeHigh", new InstantCommand(() -> m_motionControl.setState(StateConstants.kCubeHigh)));
    events.put("CubeMid", new InstantCommand(() -> m_motionControl.setState(StateConstants.kCubeMid)));
    events.put("StopDrive", new InstantCommand(()->m_drive.stop()));
    events.put("CubeIntake", (new InstantCommand(() -> m_motionControl.setState(StateConstants.kCube)).alongWith(new InstantCommand(()->m_motionControl.runCubeWhenReady(true))).alongWith(new InstantCommand(()->m_claw.setSpeed(-750)))));
    events.put("StopCubeIntake", (new InstantCommand(() -> m_motionControl.setState(StateConstants.kHome)).alongWith(new InstantCommand(()->m_motionControl.runCubeWhenReady(false))).alongWith(new InstantCommand(()->m_claw.stop())).alongWith(new InstantCommand(()->m_motionControl.forceCubeIn()))));
    events.put("WaitForArmMove", new WaitUntilCommand(m_motionControl::atSetpoint).andThen(new WaitCommand(0.05)));
    events.put("RunClawConeHigh", new WaitUntilCommand(m_motionControl::atSetpoint).andThen(new RunClaw(m_operatorBoard,m_vision, m_claw, 3)));
    events.put("RunClawConeMid", new WaitUntilCommand(m_motionControl::atSetpoint).andThen(new WaitCommand(0.2)).andThen(new RunClaw(m_operatorBoard,m_vision, m_claw, 2).withTimeout(0.25)));
    events.put("RunClawCubeMid", new RunClaw(m_operatorBoard,m_vision, m_claw, 4).withTimeout(0.75));
    events.put("RunClawCubeHigh", new RunClaw(m_operatorBoard,m_vision, m_claw, 5).withTimeout(0.75));
    events.put("RunClaw", new InstantCommand(() -> m_claw.setSpeed(2000)));
    events.put("WaitUntilReady", new WaitUntilCommand(m_motionControl::atSetpoint));
    events.put("StopClaw", new InstantCommand(() -> m_claw.setSpeed(0)));
    events.put("ConeIntake",m_autoConeIntake);
    events.put("StopConeIntake", new InstantCommand(()-> m_autoConeIntake.forceCancel()));
    events.put("ConeTransfer", m_autoConeTransfer);
    events.put("DockSimple", new RunCommand(() -> m_drive.drive(1.0, 0, 0, true, false),m_drive));
    events.put("DockNear", new Dock(m_drive, true));
    events.put("DockFar", new WaitCommand(1.0).andThen(new Dock(m_drive, false)));
    events.put("UpdateKeepAngle", new InstantCommand(()->m_drive.updateKeepAngle()));
    events.put("SetBrakeMode", new InstantCommand(()->m_drive.enableBrakeMode(true)));
    events.put("SetCoastMode", new InstantCommand(()->m_drive.enableBrakeMode(false)));
  }

  private void configureAutoChooser() {
    m_chooser.setDefaultOption("Do Nothin", doNothin);
    m_chooser.addOption("Cone Mid", new InstantCommand(() -> m_motionControl.setState(StateConstants.kConeMid)).alongWith(new WaitCommand(1.5))
    .andThen(new AutoAlign(m_drive, m_motionControl, m_claw, m_driverController, m_operatorBoard, m_vision, m_topVision, 2, 0.0)
      .alongWith(new WaitCommand(0.75).andThen(new RunClaw(m_operatorBoard,m_vision, m_claw,2)))
      .withTimeout(1.25)));
    


    for (File auto : m_autoPathFiles) {
      
      if(auto.getName().contains("Balance")){
        m_chooser.addOption(auto.getName(), 
          balanceBuilder.fullAuto(PathPlanner.loadPathGroup(auto.getName().replace(".path", ""), 4.0, 2.0)));
      }
      else if (auto.getName().contains(".path")) {
        m_chooser.addOption(
          auto.getName(), 
          autoBuilder.fullAuto(PathPlanner.loadPathGroup(auto.getName().replace(".path", ""), 4.0, 2.75))
          );
      }
    }

/*     for (File auto : m_autoPathFiles) {
      if (auto.getName().contains(".path")) {
        m_chooser.addOption(
          "Slow " + auto.getName(), 
          autoBuilder.fullAuto(PathPlanner.loadPathGroup(auto.getName().replace(".path", ""), DriveConstants.kTestMaxSpeedMetersPerSecond, DriveConstants.kTestMaxAcceleration))
        );
      }
    } */

    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public Command onEnable() {
    return new InstantCommand(()->m_motionControl.setState(StateConstants.kHome));
  }
