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

import edu.wpi.first.wpilibj.RobotController;
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

import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.utilities.ChassisAccel;
import frc.robot.utilities.MathUtils;

/**
 * Implements a swerve Drivetrain Subsystem for the Robot
 */
public class Drivetrain extends SubsystemBase {

  private double keepAngle = 0.0; // Double to store the current target keepAngle in radians
  private double timeSinceRot = 0.0; // Double to store the time since last rotation command
  private double lastRotTime = 0.0; // Double to store the time of the last rotation command
  private double timeSinceDrive = 0.0; // Double to store the time since last translation command
  private double lastDriveTime = 0.0; // Double to store the time of the last translation command

  // Create the PIDController for the Keep Angle PID
  private final PIDController m_keepAnglePID = new PIDController(DriveConstants.kKeepAnglePID[0],
      DriveConstants.kKeepAnglePID[1], DriveConstants.kKeepAnglePID[2]);

  private final Timer keepAngleTimer = new Timer(); // Creates timer used in the perform keep angle function

  private final SlewRateLimiter m_slewX = new SlewRateLimiter(7.5);
  private final SlewRateLimiter m_slewY = new SlewRateLimiter(7.5);
  private final SlewRateLimiter m_slewRot = new SlewRateLimiter(12.0);

  // Creates a swerveModule object for the front left swerve module feeding in
  // parameters from the constants file
  private final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort, DriveConstants.kFrontLeftTurningEncoderPort,
      DriveConstants.kFrontLeftOffset, DriveConstants.kFrontLeftTuningVals, DriveConstants.kUseNEO);

  // Creates a swerveModule object for the front right swerve module feeding in
  // parameters from the constants file
  private final SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort, DriveConstants.kFrontRightTurningEncoderPort,
      DriveConstants.kFrontRightOffset, DriveConstants.kFrontRightTuningVals, DriveConstants.kUseNEO);

  // Creates a swerveModule object for the back left swerve module feeding in
  // parameters from the constants file
  private final SwerveModule m_backLeft = new SwerveModule(DriveConstants.kBackLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort, DriveConstants.kBackLeftTurningEncoderPort,
      DriveConstants.kBackLeftOffset, DriveConstants.kBackLeftTuningVals, DriveConstants.kUseNEO);

  // Creates a swerveModule object for the back right swerve module feeding in
  // parameters from the constants file
  private final SwerveModule m_backRight = new SwerveModule(DriveConstants.kBackRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort, DriveConstants.kBackRightTurningEncoderPort,
      DriveConstants.kBackRightOffset, DriveConstants.kBackRightTuningVals, DriveConstants.kUseNEO);

  // Creates an ahrs gyro (NavX) on the MXP port of the RoboRIO
  private static AHRS ahrs = new AHRS(SPI.Port.kMXP);

  // Creates Odometry object to store the pose of the robot
  private final SwerveDriveOdometry m_odometry 
    = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, ahrs.getRotation2d(), getModulePositions());

  private final SwerveDriveOdometry m_autoOdometry 
    = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, ahrs.getRotation2d(), getModulePositions());

  private ChassisSpeeds m_lastDriveSpeed = new ChassisSpeeds();
  private ChassisAccel m_driveAccel = new ChassisAccel();

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

    //SmartDashboard.putNumber("Desired Drivetrain Speed", MathUtils.pythagorean(xSpeed, ySpeed));
    
    rot = m_slewRot.calculate(rot);


    if(keepAngle){
      rot = performKeepAngle(xSpeed, ySpeed, rot); // Calls the keep angle function to update the keep angle or rotate
    }
                                                 // depending on driver input


    // SmartDashboard.putNumber("xSpeed Commanded", xSpeed);
    // SmartDashboard.putNumber("ySpeed Commanded", ySpeed);

    // creates an array of the desired swerve module states based on driver command
    // and if the commands are field relative or not
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative ? secondOrderKinematics(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, ahrs.getRotation2d()))
            : secondOrderKinematics(new ChassisSpeeds(xSpeed, ySpeed, rot)));

    // normalize wheel speeds so all individual states are scaled to achievable
    // velocities
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    setModuleStates(swerveModuleStates);
  }

  @Override
  public void periodic() {

    m_driveAccel = new ChassisAccel(getChassisSpeed(),m_lastDriveSpeed,GlobalConstants.kLoopTime);

    m_lastDriveSpeed = getChassisSpeed();

    double xSpeed = getChassisSpeed().vxMetersPerSecond;
    double ySpeed = getChassisSpeed().vyMetersPerSecond;

    double speed = Math.sqrt(xSpeed*xSpeed+ySpeed*ySpeed);

    //SmartDashboard.putNumber("Speed", speed);
    //SmartDashboard.putNumber("Tilt", getTilt());

    // SmartDashboard.putNumber("Accel X", m_fieldRelAccel.ax);
    // SmartDashboard.putNumber("Accel Y", m_fieldRelAccel.ay);
    // SmartDashboard.putNumber("Alpha", m_fieldRelAccel.alpha);

     SmartDashboard.putNumber("Front Left Encoder", m_frontLeft.getTurnEncoder());
     SmartDashboard.putNumber("Front Right Encoder",m_frontRight.getTurnEncoder());
     SmartDashboard.putNumber("Back Left Encoder", m_backLeft.getTurnEncoder());
     SmartDashboard.putNumber("Back Right Encoder", m_backRight.getTurnEncoder());

     //SmartDashboard.putNumber("Front Left Speed", m_frontLeft.getState().speedMetersPerSecond);
     //SmartDashboard.putNumber("Front Right Speed",m_frontRight.getState().speedMetersPerSecond);
     //SmartDashboard.putNumber("Back Left Speed", m_backLeft.getState().speedMetersPerSecond);
     //SmartDashboard.putNumber("Back Right Speed", m_backRight.getState().speedMetersPerSecond);
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
    SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(secondOrderKinematics(chassisSpeeds));
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  public ChassisSpeeds secondOrderKinematics(ChassisSpeeds chassisSpeeds){
    Translation2d translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    Translation2d rotAdj= translation.rotateBy(new Rotation2d(-Math.PI/2.0)).times(chassisSpeeds.omegaRadiansPerSecond*0.055);

    translation = translation.plus(rotAdj);

    return new ChassisSpeeds(translation.getX(),translation.getY(),chassisSpeeds.omegaRadiansPerSecond);
  }
  
    private ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
    final double LOOP_TIME_S = 0.02;
    Pose2d futureRobotPose =
        new Pose2d(
            originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
            originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
            Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
    Twist2d twistForPose = MathUtils.log(futureRobotPose);
    ChassisSpeeds updatedSpeeds =
        new ChassisSpeeds(
            twistForPose.dx / LOOP_TIME_S,
            twistForPose.dy / LOOP_TIME_S,
            twistForPose.dtheta / LOOP_TIME_S);
    return updatedSpeeds;
  }

  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  public double getTilt() {
    return ahrs.getRoll();
    // return MathUtils.pythagorean(ahrs.getRoll(), ahrs.getPitch());
  }

  public double getTiltVel() {
    return ahrs.getRawGyroY();
  }

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

    SmartDashboard.putNumber("Robot X", position.getX());
    SmartDashboard.putNumber("Robot Y", position.getY());
    SmartDashboard.putNumber("Robot Gyro", getGyro().getRadians());

    return pose;
  }

  public Pose2d getAutoPose() {
    updateAutoOdometry();
    Pose2d pose = m_autoOdometry.getPoseMeters();
    Translation2d position = pose.getTranslation();
    return m_autoOdometry.getPoseMeters();
  }

  
  public Command toPose(Pose2d initial, Pose2d destination, Supplier<Pose2d> current) {
    ArrayList<PathPoint> points = new ArrayList<>();
    points.add(new PathPoint(initial.getTranslation(), initial.getRotation()));    
    points.add(new PathPoint(destination.getTranslation(), destination.getRotation()));

    //SmartDashboard.putNumber("Initial Pose X", initial.getX());
    //SmartDashboard.putNumber("Initial Pose Y", initial.getY());
    //SmartDashboard.putNumber("Destination Pose X", destination.getX());
    //SmartDashboard.putNumber("Destination Pose Y", destination.getY());

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
    updateKeepAngle();
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
    updateKeepAngle();
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

  public ChassisAccel getChassisAccel(){
    return m_driveAccel;
  }

  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(),
      m_backRight.getPosition()};
  }

    /**
   * Keep angle function is performed to combat drivetrain drift without the need
   * of constant "micro-adjustments" from the driver.
   * A PIDController is used to attempt to maintain the robot heading to the
   * keepAngle value. This value is updated when the robot
   * is rotated manually by the driver input
   * 
   * @return rotation command in radians/s
   * @param xSpeed is the input drive X speed command
   * @param ySpeed is the input drive Y speed command
   * @param rot    is the input drive rotation speed command
   */
  private double performKeepAngle(double xSpeed, double ySpeed, double rot) {
    double output = rot; // Output shouold be set to the input rot command unless the Keep Angle PID is
                         // called
    if (Math.abs(rot) >= DriveConstants.kMinRotationCommand) { // If the driver commands the robot to rotate set the
                                                               // last rotate time to the current time
      lastRotTime = keepAngleTimer.get();
    }
    if (Math.abs(xSpeed) >= DriveConstants.kMinTranslationCommand
        || Math.abs(ySpeed) >= DriveConstants.kMinTranslationCommand) { // if driver commands robot to translate set the
                                                                        // last drive time to the current time
      lastDriveTime = keepAngleTimer.get();
    }
    timeSinceRot = keepAngleTimer.get() - lastRotTime; // update variable to the current time - the last rotate time
    timeSinceDrive = keepAngleTimer.get() - lastDriveTime; // update variable to the current time - the last drive time
    if (timeSinceRot < 0.5) { // Update keepAngle up until 0.5s after rotate command stops to allow rotation
                              // move to finish
      keepAngle = getGyro().getRadians();
    } else if (Math.abs(rot) < DriveConstants.kMinRotationCommand && timeSinceDrive < 0.25) { // Run Keep angle pid
                                                                                              // until 0.75s after drive
                                                                                              // command stops to combat
                                                                                              // decel drift
      output = m_keepAnglePID.calculate(getGyro().getRadians(), keepAngle); // Set output command to the result of the
                                                                            // Keep Angle PID
    }
    return output;
  }

  public void updateKeepAngle() {
    keepAngle = getGyro().getRadians();
  }

}
