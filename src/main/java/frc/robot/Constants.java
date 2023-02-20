// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
  }

  public static final class CurrentLimit {
    public static final int kTranslation = 80;
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
    public static final MotionControlState kHome = new MotionControlState(-17.75, 0.0, -18.0, 12.0, 0.0);
    public static final MotionControlState kGrab = new MotionControlState(66.86, 0.0, -11.5, 54.88, 0.0);
    public static final MotionControlState kShoot = new MotionControlState(63, 0, -1.5, 45, 0.0);
    public static final MotionControlState kFloor = new MotionControlState(26.0, 0, -28.0, 33.5, 0.0);

  }

  public static final class ModuleConstants {
    private static final double kTranslationGearRatio = 8.33333333*0.750*0.830769231; // Overall gear ratio of the swerve module
    private static final double kWheelDiameter = 0.095; // Wheel Diameter in meters, may need to be
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

    public static final double kFrontLeftOffset = -0.634+Math.PI; // Encoder Offset in Radians
    public static final double kFrontRightOffset = -2.437; // Encoder Offset in Radians
    public static final double kBackLeftOffset = +0.071-1.560; // Encoder Offset in Radians
    public static final double kBackRightOffset = -3.360; // Encoder Offset in Radians

    public static final double[] kFrontLeftTuningVals = { 0.0150*0.5, 0.2850*0.97, 0.15, 0 }; // {Static Gain, FeedForward,
                                                                                     // Proportional Gain, ModuleID for
                                                                                     // Tuning}
    public static final double[] kFrontRightTuningVals = { 0.0150*0.5, 0.2850*0.97, 0.15, 1 }; // {Static Gain, FeedForward,
                                                                                      // Proportional Gain, ModuleID for
                                                                                      // Tuning}
    public static final double[] kBackLeftTuningVals = { 0.0150*0.5, 0.2850*0.97, 0.15, 2 }; // {Static Gain, FeedForward,
                                                                                    // Proportional Gain, ModuleID for
                                                                                    // Tuning}
    public static final double[] kBackRightTuningVals = { 0.0150*0.5, 0.2850*0.97, 0.15, 3 }; // {Static Gain, FeedForward,
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
    public static final double kMaxAngularSpeed = 1.5*Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    public static final double kMaxAngularAccel = 1.5*Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly

    public static final double kInnerDeadband = 0.08; // This value should exceed the maximum value the analog stick may
                                                      // read when not in use (Eliminates "Stick Drift")
    public static final double kOuterDeadband = 0.98; // This value should be lower than the analog stick X or Y reading
                                                      // when aimed at a 45deg angle (Such that X and Y are are
                                                      // maximized simultaneously)

    public static final double[] kKeepAnglePID = { 0.300, 0, 0 }; // Defines the PID values for the keep angle PID

  }

  public static final class FieldConstants {
    public static final double kFieldWidth = 8.02;
    public static final double kFieldLength = 16.54;
    public static final double[] kScoringZone = {0.00, 3.33, 0.00, 5.50};
    public static final double[] kForeField = {3.34, 6.00, 0.00, 8.02};
    public static final double[] kMidField = {6.01, kFieldLength / 2, 0.00, 8.02};
    public static final double[] kLoadingZone = {0.00, 3.33, 5.51, 8.02};
    /**
     * True = Blue --- False = Red
     */
    public static final boolean kAlliance = DriverStation.getAlliance() == Alliance.Blue;
  }

  public static final class ArmsConstants {
    public static final double kDefaultElevator = -1.0;
    public static final TrapezoidProfile.Constraints kElevatorConstraints = new TrapezoidProfile.Constraints(250, 50);
    public static final double kMinElevator = -29;
    public static final double kMaxElevator = -1;

    public static final double kDefaultArm = 0.0;
    public static final TrapezoidProfile.Constraints kArmConstraints = new TrapezoidProfile.Constraints(250, 50);
    public static final double kMinArm = -25;
    public static final double kMaxArm = 70;

    public static final double kDefaultWrist = 0.0;
    public static final TrapezoidProfile.Constraints kWristConstraints = new TrapezoidProfile.Constraints(250, 50);
    public static final double kMinWrist = -30;
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
    public static final double kExtendedCone = 30.0;
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
    public static final Pose3d kLimelightToRobot = new Pose3d();
  }

  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.6; // Sets a voltage compensation value ideally 12.6V
    public static final int PCHID = 20;
    public static final int PDHID = 24;
    public static final double kLoopTime = 0.020;
  }
}
