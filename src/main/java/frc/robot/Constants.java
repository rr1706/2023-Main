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
    public static final int kOperatorControllerPort = 1;
    public static final int kOperatorBoardPort = 2;  }

  public static final class CurrentLimit {
    public static final int kTranslation = 65;
    public static final int kRotation = 30;
    public static final int kCubeExt = 25;
    public static final int kConeExt = 25;
    public static final int kCube = 25;
    public static final int kCone = 25;
    public static final int kClaw = 30;
    public static final int kElevator = 25;
    public static final int kWrist = 25;
    public static final int kArm = 25;
  }

  public static final class StateConstants{
    public static final double armAdj = 1.5;
    public static final MotionControlState kHome = new MotionControlState(-17.95+armAdj, 0.0, -18.25, 11.2, 0.0);
    public static final MotionControlState kGrab = new MotionControlState(64.9+armAdj, 0.0, -12.3, 54.08, 0.0);
    public static final MotionControlState kShoot = new MotionControlState(62.8+armAdj, 0, -1.5, 44.2, 0.0);
    public static final MotionControlState kMidShoot = new MotionControlState(44.8+armAdj, 0, -5.0, 34.2, 0.0);
    public static final MotionControlState kFloor = new MotionControlState(23.8+armAdj, 0, -28.2, 32.7, 0.0);
    public static final MotionControlState kCube = new MotionControlState(22.64+armAdj, 22, -9.79, 56.33, 0.0);
    public static final MotionControlState kConeIntake = new MotionControlState(-24.2+armAdj, 0, -14.5, -30.8, 32.0);
    public static final MotionControlState kLow = new MotionControlState(23.8+armAdj, 0, -25.0, 29.2, 0.0);
    public static final MotionControlState kConeMid = new MotionControlState(44.8+armAdj, 0, -5.0, 34.2, 0.0);
    public static final MotionControlState kConeHigh =  new MotionControlState(62.8+armAdj, 0, -1.5, 44.2, 0.0);
    public static final MotionControlState kCubeMid = new MotionControlState(7.8+armAdj, 0, -7.5, 16, 0.0);
    public static final MotionControlState kCubeHigh =  new MotionControlState(41.8+armAdj, 0, -5.0, 33, 0.0);

  }

  public static final class ModuleConstants {
    private static final double kTranslationGearRatio = 5.1923; // Overall gear ratio of the swerve module
    private static final double kWheelDiameter = 0.09786; // Wheel Diameter in meters, may need to be
                                                                         // experimentally determined due to compliance
                                                                         // of floor/tread material

    public static final double kVelocityFactor = (1.0 / kTranslationGearRatio / 60.0) * kWheelDiameter * Math.PI; 

    public static final double[] kTurnPID = { 0.800, 0, 0 }; // Defines the PID values for rotation of the serve
                                                             // modules, should show some minor oscillation when no
                                                             // weight is loaded on the modules
  }

  public static final class DriveConstants {
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

    public static final double kFrontLeftOffset = -0.8914; // Encoder Offset in Radians
    public static final double kFrontRightOffset = -3.8591+Math.PI; // Encoder Offset in Radians
    public static final double kBackLeftOffset = -1.991+0.7833; // Encoder Offset in Radians
    public static final double kBackRightOffset = -3.7096+Math.PI; // Encoder Offset in Radians

    public static final double[] kFrontLeftTuningVals = { 0.0075, 0.169, 0.15, 0 }; // {Static Gain, FeedForward,
                                                                                     // Proportional Gain, ModuleID for
                                                                                     // Tuning}
    public static final double[] kFrontRightTuningVals = { 0.0075, 0.169, 0.15, 1 }; // {Static Gain, FeedForward,
                                                                                      // Proportional Gain, ModuleID for
                                                                                      // Tuning}
    public static final double[] kBackLeftTuningVals = { 0.0075, 0.169, 0.15, 2 }; // {Static Gain, FeedForward,
                                                                                    // Proportional Gain, ModuleID for
                                                                                    // Tuning}
    public static final double[] kBackRightTuningVals = { 0.0075, 0.169, 0.15, 3 }; // {Static Gain, FeedForward,
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
    public static final double kMaxSpeedMetersPerSecond = 5.0; // Maximum Sustainable Drivetrain Speed under Normal
                                                                // Conditions & Battery, Robot will not exceed this
                                                                // speed in closed loop control
    public static final double kTestMaxAcceleration = 0.8;
    public static final double kTestMaxSpeedMetersPerSecond = 1.2;
     
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
    public static final boolean kAlliance = DriverStation.getAlliance() == Alliance.Blue;
  }

  public static final class ArmsConstants {
    public static final double kDefaultElevator = -1.0;
    public static final TrapezoidProfile.Constraints kElevatorConstraints = new TrapezoidProfile.Constraints(250, 50);
    public static final double kMinElevator = -30;
    public static final double kMaxElevator = -1;

    public static final double kDefaultArm = 0.0;
    public static final TrapezoidProfile.Constraints kArmConstraints = new TrapezoidProfile.Constraints(250, 50);
    public static final double kMinArm = -25;
    public static final double kMaxArm = 70;

    public static final double kDefaultWrist = 0.0;
    public static final TrapezoidProfile.Constraints kWristConstraints = new TrapezoidProfile.Constraints(250, 50);
    public static final double kMinWrist = -35;
    public static final double kMaxWrist = 56;

    public static final double kArmLength = 0.0;
    public static final double kWristLength = 0.0;

    public static final int[] kArmMotors = {16, 17};
    public static final int[] kElevatorMotors = {12, 13};
    public static final int kWristMotor = 15;
  }

  public static final class IntakeConstants {
    public static final double kDefaultCone = 0.0;
    public static final TrapezoidProfile.Constraints kConeConstraints = new TrapezoidProfile.Constraints(0.0, 0.0);
    public static final double kExtendedCone = 33.0;
    public static final double kRetractedCone = 0.0;
    
    public static final double kDefaultCube = 0.0;
    public static final TrapezoidProfile.Constraints kCubeConstraints = new TrapezoidProfile.Constraints(0.0, 0.0);
    public static final double kExtendedCube = 22.0;
    public static final double kRetractedCube = 0.0;

    public static final int[] kConeMotors = {9, 14};
    public static final int[] kCubeMotors = {10, 11};

    public static final double kCubeP = 0.0001;
    public static final double kCubeFF = 0.000009;
    
    public static final double kConeP = 0.0001;
    public static final double kConeFF = 0.000009;
  }

  public static final class VisionConstants {
    public static final double kPoseErrorAcceptance = 1.0; // How much error there can be between current stimated pose and vision pose in meters
  }

  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.6; // Sets a voltage compensation value ideally 12.6V
    public static final int PCHID = 20;
    public static final int PDHID = 24;
    public static final double kLoopTime = 0.020;
  }
}
