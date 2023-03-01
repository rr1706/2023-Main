// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import java.beans.DesignMode;
import java.io.PipedInputStream;
import java.util.ArrayList;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import frc.robot.Constants.*;
import frc.robot.utilities.MathUtils;

/**
 * Implements a swerve Drivetrain Subsystem for the Robot
 */
public class Drivetrain extends SubsystemBase {

  // Create the PIDController for the Keep Angle PID
  private final PIDController m_keepAnglePID = new PIDController(DriveConstants.kKeepAnglePID[0],
      DriveConstants.kKeepAnglePID[1], DriveConstants.kKeepAnglePID[2]);

  private final Timer keepAngleTimer = new Timer(); // Creates timer used in the perform keep angle function

  private final SlewRateLimiter m_slewX = new SlewRateLimiter(8.0);
  private final SlewRateLimiter m_slewY = new SlewRateLimiter(8.0);
  private final SlewRateLimiter m_slewRot = new SlewRateLimiter(12.5);

  // Creates a swerveModule object for the front left swerve module feeding in
  // parameters from the constants file
  private final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort, DriveConstants.kFrontLeftTurningEncoderPort,
      DriveConstants.kFrontLeftOffset, DriveConstants.kFrontLeftTuningVals);

  // Creates a swerveModule object for the front right swerve module feeding in
  // parameters from the constants file
  private final SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort, DriveConstants.kFrontRightTurningEncoderPort,
      DriveConstants.kFrontRightOffset, DriveConstants.kFrontRightTuningVals);

  // Creates a swerveModule object for the back left swerve module feeding in
  // parameters from the constants file
  private final SwerveModule m_backLeft = new SwerveModule(DriveConstants.kBackLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort, DriveConstants.kBackLeftTurningEncoderPort,
      DriveConstants.kBackLeftOffset, DriveConstants.kBackLeftTuningVals);

  // Creates a swerveModule object for the back right swerve module feeding in
  // parameters from the constants file
  private final SwerveModule m_backRight = new SwerveModule(DriveConstants.kBackRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort, DriveConstants.kBackRightTurningEncoderPort,
      DriveConstants.kBackRightOffset, DriveConstants.kBackRightTuningVals);

  // Creates an ahrs gyro (NavX) on the MXP port of the RoboRIO
  private static AHRS ahrs = new AHRS(SPI.Port.kMXP);

  // Creates Odometry object to store the pose of the robot
  private final SwerveDriveOdometry m_odometry 
    = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, ahrs.getRotation2d(), getModulePositions());

  private final SwerveDriveOdometry m_autoOdometry 
    = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, ahrs.getRotation2d(), getModulePositions());


  /**
   * Constructs a Drivetrain and resets the Gyro and Keep Angle parameters
   */
  public Drivetrain() {
    keepAngleTimer.reset();
    keepAngleTimer.start();
    m_keepAnglePID.enableContinuousInput(-Math.PI, Math.PI);
    m_odometry.resetPosition(ahrs.getRotation2d(), getModulePositions(), new Pose2d());
    ahrs.reset();
    ahrs.calibrate();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean keepAngle) {
    
    xSpeed = m_slewX.calculate(xSpeed);
    ySpeed = m_slewY.calculate(ySpeed);
    rot = m_slewRot.calculate(rot);

    // SmartDashboard.putNumber("xSpeed Commanded", xSpeed);
    // SmartDashboard.putNumber("ySpeed Commanded", ySpeed);

    // creates an array of the desired swerve module states based on driver command
    // and if the commands are field relative or not
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, ahrs.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    // normalize wheel speeds so all individual states are scaled to achievable
    // velocities
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    setModuleStates(swerveModuleStates);
  }

  @Override
  public void periodic() {

    double xSpeed = getChassisSpeed().vxMetersPerSecond;
    double ySpeed = getChassisSpeed().vyMetersPerSecond;

    double speed = Math.sqrt(xSpeed*xSpeed+ySpeed*ySpeed);

    SmartDashboard.putNumber("Speed", speed);

    // SmartDashboard.putNumber("Accel X", m_fieldRelAccel.ax);
    // SmartDashboard.putNumber("Accel Y", m_fieldRelAccel.ay);
    // SmartDashboard.putNumber("Alpha", m_fieldRelAccel.alpha);

     SmartDashboard.putNumber("Front Left Encoder", m_frontLeft.getTurnEncoder());
     SmartDashboard.putNumber("Front Right Encoder",m_frontRight.getTurnEncoder());
     SmartDashboard.putNumber("Back Left Encoder", m_backLeft.getTurnEncoder());
     SmartDashboard.putNumber("Back Right Encoder", m_backRight.getTurnEncoder());

    // Update swerve drive odometry periodically so robot pose can be tracked
    updateOdometry();

    // Calls get pose function which sends the Pose information to the
    // SmartDashboard
    getPose();
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  //public double getTilt() {
   // return MathUtils.pythagorean(ahrs.get, ahrs.getPitch());
//  }

  /**
   * Updates odometry for the swerve drivetrain. This should be called
   * once per loop to minimize error.
   */  
  public void updateOdometry() {
    m_odometry.update(ahrs.getRotation2d(), getModulePositions());
  }

  public void updateAutoOdometry() {
    m_autoOdometry.update(ahrs.getRotation2d(), getModulePositions());
  }

  /**
   * Function to retrieve latest robot gyro angle.
   * 
   * @return Rotation2d object containing Gyro angle
   */
  public Rotation2d getGyro() {
    return ahrs.getRotation2d();
  }

  /**
   * Function created to retreieve and push the robot pose to the SmartDashboard
   * for diagnostics
   * 
   * @return Pose2d object containing the X and Y position and the heading of the
   *         robot.
   */
  public Pose2d getPose() {
    Pose2d pose = m_odometry.getPoseMeters();
    Translation2d position = pose.getTranslation();

    return m_odometry.getPoseMeters();
  }

  public Pose2d getAutoPose() {
    updateAutoOdometry();
    Pose2d pose = m_autoOdometry.getPoseMeters();
    Translation2d position = pose.getTranslation();
    SmartDashboard.putNumber("Auto X", position.getX());
    SmartDashboard.putNumber("Auto Y", position.getY());
    return m_autoOdometry.getPoseMeters();
  }

  
  public Command toPose(Pose2d initial, Pose2d destination, Supplier<Pose2d> current) {
    ArrayList<PathPoint> points = new ArrayList<>();
    points.add(new PathPoint(initial.getTranslation(), initial.getRotation()));    
    points.add(new PathPoint(destination.getTranslation(), destination.getRotation()));

    SmartDashboard.putNumber("Initial Pose X", initial.getX());
    SmartDashboard.putNumber("Initial Pose Y", initial.getY());
    SmartDashboard.putNumber("Destination Pose X", destination.getX());
    SmartDashboard.putNumber("Destination Pose Y", destination.getY());

    return new PPSwerveControllerCommand(
      PathPlanner.generatePath(new PathConstraints(DriveConstants.kTestMaxSpeedMetersPerSecond, DriveConstants.kTestMaxAcceleration), points),
      current,
      new PIDController(0.0, 0.0, 0.0),
      new PIDController(0.0, 0.0, 0.0),
      //new PIDController(ModuleConstants.kTurnPID[0], ModuleConstants.kTurnPID[1], ModuleConstants.kTurnPID[2]),
      new PIDController(0.0, 0.0, 0.0),
      this::setModuleStates);
  }

  /**
   * Resets the odometry and gyro to the specified pose.
   *
   * @param pose in which to set the odometry and gyro.
   */
  public void resetOdometry(Pose2d pose) {
    ahrs.reset();
    ahrs.setAngleAdjustment(pose.getRotation().getDegrees());
    m_odometry.resetPosition(ahrs.getRotation2d().times(-1.0), getModulePositions(), pose);
    m_autoOdometry.resetPosition(ahrs.getRotation2d().times(-1.0), getModulePositions(), pose);
  }

  public void setPose(Pose2d pose){
    m_odometry.resetPosition(ahrs.getRotation2d().times(-1.0), getModulePositions(), pose);
  }


    /**
   * Resets the gyro to the given angle
   * 
   * @param angle the angle of the robot to reset to
   */
  public void resetOdometry(Rotation2d angle) {
    ahrs.reset();
    ahrs.setAngleAdjustment(angle.getDegrees());
    Pose2d pose = new Pose2d(getPose().getTranslation(), angle);
    m_odometry.resetPosition(ahrs.getRotation2d().times(-1.0), getModulePositions(), pose);  }

  /**
   * Converts the 4 swerve module states into a chassisSpeed by making use of the
   * swerve drive kinematics.
   * 
   * @return ChassisSpeeds object containing robot X, Y, and Angular velocity
   */
  public ChassisSpeeds getChassisSpeed() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }

  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(),
      m_backRight.getPosition()};
  }

}