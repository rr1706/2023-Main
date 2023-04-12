// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utilities.MotionControlState;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 2;
    public static final int kOperatorBoardPort = 1;  }

  public static final class CurrentLimit {
    public static final int kTranslation = 57;
    public static final int kRotation = 20;
    public static final int kCubeExt = 20;
    public static final int kConeExt = 20;
    public static final int kCube = 20;
    public static final int kCone = 20;
    public static final int kClaw = 40;
    public static final int kElevator = 20;
    public static final int kWrist = 20;
    public static final int kArm = 20;
  }

  public static final class StateConstants{
    public static final double armAdj = 0.0;
    public static final double armAdjRed = armAdj;
    public static final double armAdjBlue = armAdj;
    public static final MotionControlState kHome = new MotionControlState(-12.0, 0.0, -16.2, 12.0*1.40, 5.0);
    public static final MotionControlState kGrab = new MotionControlState(76.5, 0.0, -13.45, 64.5*1.40, 5.0);
    public static final MotionControlState kGrabRed = new MotionControlState(76.5, 0.0, -13.45, 64.5*1.40, 5.0);
    public static final MotionControlState kGrabBlue = new MotionControlState(76.5, 0.0, -13.45, 64.5*1.40, 5.0);
    public static final MotionControlState kFloor = new MotionControlState(32.4, 0, -28.35, 40.0*1.40, 5.0);
    public static final MotionControlState kCube = new MotionControlState(16.8, 18, -6.7, 59.0*1.40, 8.0);
    public static final MotionControlState kConeIntake = new MotionControlState(-14.0, 0, -1.5, -24.0*1.40, 37.0);
    public static final MotionControlState kConeIntakeIn = new MotionControlState(-14.0, 0, -2.0, -60.0, 2.0);
    public static final MotionControlState kConeIntakeIn2 = new MotionControlState(-14.0, 0, -3.0, -60.0, 2.0);
    public static final MotionControlState kLow = new MotionControlState(-12.0, 0.0, -16.2, 12.0*1.40, 5.0);
    public static final MotionControlState kConeMid = new MotionControlState(63.0, 0, -9.5, 48.0*1.40, 5.0);
    public static final MotionControlState kConeHigh =  new MotionControlState(73.6, 0, -1.5, 52.0*1.40, 5.0);
    public static final MotionControlState kCubeMid = new MotionControlState(9.04, 0, -6.7, 18.0*1.40, 5.0);
    public static final MotionControlState kCubeHigh =  new MotionControlState(46.0, 0, -4.2,40.0*1.40, 5.0);
    public static final MotionControlState kRevCubeMidInt = new MotionControlState(10.24, 0, -1.5, 11.0*1.40, 5.0);
    public static final MotionControlState kRevCubeMidMid = new MotionControlState(10.24, 0, -1.5, -22.0*1.40, 5.0);
    public static final MotionControlState kRevCubeMidFin = new MotionControlState(-14.0, 0, -1.5, -18.0*1.40, 5.0);
    public static final MotionControlState kStart = new MotionControlState(0, 0, -1.5, 0, 5.0);
    public static final MotionControlState kCoolThing = new MotionControlState(73.28,18,-11.5,62.5*1.40,37);
  }


  public static final class ModuleConstants {
    private static final double kTranslationGearRatio = 5.6933; // Overall gear ratio of the swerve module
    public static final double kRotationGearRatio = 18.0;
    private static final double kWheelDiameter = 0.09645; // Wheel Diameter in meters, may need to be
                                                                         // experimentally determined due to compliance
                                                                         // of floor/tread material

    public static final double kVelocityFactor = (1.0 / kTranslationGearRatio / 60.0) * kWheelDiameter * Math.PI; 
    public static final double kNEOSteerP = 3.0;
    public static final double[] kTurnPID = { 1.00, 0, 0 }; // Defines the PID values for rotation of the serve
                                                             // modules, should show some minor oscillation when no
                                                             // weight is loaded on the modules
  }

  public static final class DriveConstants {
    public static final boolean kUseNEO = false;

  public static final int kFrontLeftDriveMotorPort = 1; // CANID of the Translation SparkMAX
    public static final int kFrontRightDriveMotorPort = 3; // CANID of the Translation SparkMAX
    public static final int kBackLeftDriveMotorPort = 5; // CANID of the Translation SparkMAX
    public static final int kBackRightDriveMotorPort = 7; // CANID of the Translation SparkMAX

    public static final int kFrontLeftTurningMotorPort = 2; // CANID of the Rotation SparkMAX
    public static final int kFrontRightTurningMotorPort = 4; // CANID of the Rotation SparkMAX
    public static final int kBackLeftTurningMotorPort = 6; // CANID of the Rotation SparkMAX
    public static final int kBackRightTurningMotorPort = 8; // CANID of the Rotation SparkMAX

    public static final int kFrontLeftTurningEncoderPort = 0; // Analog Port of the Module Absolute Encoder
    public static final int kFrontRightTurningEncoderPort = 1; // Analog Port of the Module Absolute Encoder
    public static final int kBackLeftTurningEncoderPort = 2; // Analog Port of the Module Absolute Encoder
    public static final int kBackRightTurningEncoderPort = 3; // Analog Port of the Module Absolute Encoder

    public static final double kFrontLeftOffset = -0.9832*2*Math.PI+12.335+0.01-2.445; // Encoder Offset in Radians
    public static final double kFrontRightOffset = -0.6225*2*Math.PI+7.820+0.01-1.756; // Encoder Offset in Radians
    public static final double kBackLeftOffset = -0.1119*2*Math.PI+1.419-0.03; // Encoder Offset in Radians
    public static final double kBackRightOffset = -0.5598*2*Math.PI+7.046+0.02-1.356; // Encoder Offset in Radians

    public static final double[] kFrontLeftTuningVals = { 0.015, 0.19, 0.15, 0 }; // {Static Gain, FeedForward,
                                                                                     // Proportional Gain, ModuleID for
                                                                                     // Tuning}
    public static final double[] kFrontRightTuningVals = { 0.015, 0.19, 0.15, 1 }; // {Static Gain, FeedForward,
                                                                                      // Proportional Gain, ModuleID for
                                                                                      // Tuning}
    public static final double[] kBackLeftTuningVals = { 0.015, 0.19, 0.15, 2 }; // {Static Gain, FeedForward,
                                                                                    // Proportional Gain, ModuleID for
                                                                                    // Tuning}
    public static final double[] kBackRightTuningVals = { 0.015, 0.19, 0.15, 3 }; // {Static Gain, FeedForward,
                                                                                     // Proportional Gain, ModuleID for
                                                                                     // Tuning}

    // NOTE: 2910 Swerve the wheels are not directly under the center of rotation
    // (Take into consideration when measuring)
    public static final double kWheelBaseWidth = 0.4699; // Center distance in meters between right and left wheels on
                                                         // robot
    public static final double kWheelBaseLength = 0.5461; // Center distance in meters between front and back wheels on
                                                          // robot

    // Because the swerve modules poisition does not change, define a constant
    // SwerveDriveKinematics for use throughout the code
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBaseLength / 2, kWheelBaseWidth / 2),
        new Translation2d(kWheelBaseLength / 2, -kWheelBaseWidth / 2),
        new Translation2d(-kWheelBaseLength / 2, kWheelBaseWidth / 2),
        new Translation2d(-kWheelBaseLength / 2, -kWheelBaseWidth / 2));

    public static final double kMaxAcceleration = 3.0;
    public static final double kMaxSpeedMetersPerSecond = 4.5; // Maximum Sustainable Drivetrain Speed under Normal
                                                                // Conditions & Battery, Robot will not exceed this
                                                                // speed in closed loop control
    public static final double kTestMaxAcceleration = 1.0;
    public static final double kTestMaxSpeedMetersPerSecond = 1.0;
     
    public static final double kMaxAngularSpeed = Math.PI*1.5; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    public static final double kMaxAngularAccel = 1.5*Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly

    public static final double kInnerDeadband = 0.10; // This value should exceed the maximum value the analog stick may
                                                      // read when not in use (Eliminates "Stick Drift")
    public static final double kOuterDeadband = 0.98; // This value should be lower than the analog stick X or Y reading
                                                      // when aimed at a 45deg angle (Such that X and Y are are
                                                      // maximized simultaneously)

    public static final double[] kKeepAnglePID = { 0.300, 0, 0 }; // Defines the PID values for the keep angle PID

    private static final SwerveModuleState[] kLockedWheelsHelper = kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 1.0));
    public static final SwerveModuleState[] kLockedWheels = {
      new SwerveModuleState(0.0, kLockedWheelsHelper[0].angle.rotateBy(new Rotation2d(Math.PI/2))),
      new SwerveModuleState(0.0, kLockedWheelsHelper[1].angle.rotateBy(new Rotation2d(Math.PI/2))),
      new SwerveModuleState(0.0, kLockedWheelsHelper[2].angle.rotateBy(new Rotation2d(Math.PI/2))),
      new SwerveModuleState(0.0, kLockedWheelsHelper[3].angle.rotateBy(new Rotation2d(Math.PI/2)))
    };


    // Minimum allowable rotation command (in radians/s) assuming user input is
    // squared using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinRotationCommand = DriveConstants.kMaxAngularSpeed
        * Math.pow(DriveConstants.kInnerDeadband, 2);
    // Minimum allowable tranlsation command (in m/s) assuming user input is squared
    // using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinTranslationCommand = DriveConstants.kMaxSpeedMetersPerSecond
        * Math.pow(DriveConstants.kInnerDeadband, 2);

  }

  public static final class FieldConstants {
    public static final double kFieldWidth = 8.02;
    public static final double kFieldLength = 16.54;

    /**
     * Valid y-positions for scoring
     */
    public static final double[] kScoringPositions = {4.94, 4.38, 3.82, 3.26, 2.70, 2.13, 1.58, 1.01, 0.46};
    public static final double kScoringTolerance = 0.04;

    public static final Translation2d[] kScoringZone = {new Translation2d(1.83, 0.00), new Translation2d(1.87, 5.00)};
    public static final Translation2d[] kScoringPrepZone = {new Translation2d(1.87, 0.00), new Translation2d(2.45, 5.00)};
    public static final Translation2d[] kLoadingZone = {new Translation2d(13.00, 6.07), new Translation2d(15.77, 8.02)};
    public static final Translation2d[] kMidClose = {new Translation2d(2.45, 0.00), new Translation2d(kFieldLength / 2, 8.02)};
    public static final Translation2d[] kMidFar = {new Translation2d(kFieldLength / 2, 0.00), new Translation2d(13.00, 8.02)};

    /**
     * True = Blue --- False = Red
     */
    //public static final boolean kAlliance = DriverStation.getAlliance() == Alliance.Blue;
  }

  public static final class ArmsConstants {
    public static final double kDefaultElevator = -1.0;
    public static final TrapezoidProfile.Constraints kElevatorConstraints = new TrapezoidProfile.Constraints(250, 50);
    public static final double kMinElevator = -30;
    public static final double kMaxElevator = -1;
    public static final double kElevatorToCentimeters = 0.0;

    public static final double kDefaultArm = 0.0;
    public static final TrapezoidProfile.Constraints kArmConstraints = new TrapezoidProfile.Constraints(250, 50);
    public static final double kMinArm = -15.0;
    public static final double kMaxArm = 78.4;
    public static final double kArmToRadians = 0.0;
    public static final double kArmLength = 0.0;

    public static final double kDefaultWrist = 0.0;
    public static final TrapezoidProfile.Constraints kWristConstraints = new TrapezoidProfile.Constraints(250, 50);
    public static final double kMinWrist = -61;
    public static final double kMaxWrist = 65*1.40;
    public static final double kWristToRadians = 0.0;
    public static final double kWristLength = 0.0;

    public static final double kSmartMotionTolerance = 100; // The higher the value, the lower the tolerance
    public static final double kShotAccelComp = 0.0;

    public static final int[] kArmMotors = {16, 17};
    public static final int[] kElevatorMotors = {12, 13};
    public static final int kWristMotor = 15;

    public static final int kArmAbsEncoder = 4;
  }

  public static final class IntakeConstants {
    public static final double kDefaultCone = 8.0;
    public static final TrapezoidProfile.Constraints kConeConstraints = new TrapezoidProfile.Constraints(0.0, 0.0);
    public static final double kExtendedCone = 37.0;
    public static final double kRetractedCone = 2.0;
    
    public static final double kDefaultCube = 0.0;
    public static final TrapezoidProfile.Constraints kCubeConstraints = new TrapezoidProfile.Constraints(0.0, 0.0);
    public static final double kExtendedCube = 18.0;
    public static final double kRetractedCube = 0.0;

    public static final int[] kConeMotors = {9, 14};
    public static final int[] kCubeMotors = {10, 11};

    public static final double kCubeP = 0.0001;
    public static final double kCubeFF = 0.000009;
    
    public static final double kConeP = 0.0001;
    public static final double kConeFF = 0.000009;
  }

  public static final class VisionConstants {
    public static final double kPoseErrorAcceptance = 2.0; // How much error there can be between current stimated pose and vision pose in meters
  }

  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.6; // Sets a voltage compensation value ideally 12.6V
    public static final int PCHID = 20;
    public static final int PDHID = 24;
    public static final double kLoopTime = 0.020;
  }
}
