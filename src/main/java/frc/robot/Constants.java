// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
    /**
     * True = Red --- False = Blue
     */
    public static final boolean kAlliance = DriverStation.getAlliance() == Alliance.Red;
  }

  public static class CurrentLimit {
    public static final int kRotation = 25;
    public static final int kTranslation = 30;
  }

  public static class ModuleConstants {
    public static final double kTranslationRampRate = 3.0; // Units of %power/s, ie 4.0 means it takes 0.25s to reach
                                                           // 100% power from 0%
    private static final double kTranslationGearRatio = 8.33333333; // Overall gear ratio of the swerve module
    private static final double kWheelDiameter = 0.0986 * 0.960; // Wheel Diameter in meters, may need to be
                                                                         // experimentally determined due to compliance
                                                                         // of floor/tread material

    public static final double kVelocityFactor = (1.0 / kTranslationGearRatio / 60.0) * kWheelDiameter * Math.PI; // Calculates
                                                                                                                  // the
                                                                                                                  // conversion
                                                                                                                  // factor
                                                                                                                  // of
                                                                                                                  // RPM
                                                                                                                  // of
                                                                                                                  // the
                                                                                                                  // translation
                                                                                                                  // motor
                                                                                                                  // to
                                                                                                                  // m/s
                                                                                                                  // at
                                                                                                                  // the
                                                                                                                  // floor

    // NOTE: You shoulds ALWAYS define a reasonable current limit when using
    // brushless motors
    // due to the extremely high stall current avaialble

    public static final double[] kTurnPID = { 0.600, 0, 0 }; // Defines the PID values for the rotation of the swerve
                                                             // modules, should show some minor oscillation when no
                                                             // weight is loaded on the modules
  }

  public static class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 19; // CANID of the Translation SparkMAX
    public static final int kFrontRightDriveMotorPort = 4; // CANID of the Translation SparkMAX
    public static final int kBackLeftDriveMotorPort = 6; // CANID of the Translation SparkMAX
    public static final int kBackRightDriveMotorPort = 2; // CANID of the Translation SparkMAX

    public static final int kFrontLeftTurningMotorPort = 18; // CANID of the Rotation SparkMAX
    public static final int kFrontRightTurningMotorPort = 3; // CANID of the Rotation SparkMAX
    public static final int kBackLeftTurningMotorPort = 7; // CANID of the Rotation SparkMAX
    public static final int kBackRightTurningMotorPort = 1; // CANID of the Rotation SparkMAX

    public static final int kFrontLeftTurningEncoderPort = 0; // Analog Port of the Module Absolute Encoder
    public static final int kFrontRightTurningEncoderPort = 1; // Analog Port of the Module Absolute Encoder
    public static final int kBackLeftTurningEncoderPort = 3; // Analog Port of the Module Absolute Encoder
    public static final int kBackRightTurningEncoderPort = 2; // Analog Port of the Module Absolute Encoder

    public static final double kFrontLeftOffset = 1.3632+0.0122; // Encoder Offset in Radians
    public static final double kFrontRightOffset = -0.3005-0.195-0.0115; // Encoder Offset in Radians
    public static final double kBackLeftOffset = 0.3192+0.0332; // Encoder Offset in Radians
    public static final double kBackRightOffset = -5.9208+0.008; // Encoder Offset in Radians

    // Drive motor PID is best done on the roboRIO currently as the SparkMAX does
    // not allow for static gain values on the PID controller,
    // these are necessary to have high accuracy when moving at extremely low RPMs
    // public static final double[] kFrontLeftTuningVals = {0.0120,0.2892,0.25,0};
    // //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    // public static final double[] kFrontRightTuningVals = {0.0092,0.2835,0.25,1};
    // //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    // public static final double[] kBackLeftTuningVals = {0.0142,0.2901,0.25,2};
    // //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    // public static final double[] kBackRightTuningVals = {0.0108,0.2828,0.25,3};
    // //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}

    public static final double[] kFrontLeftTuningVals = { 0.0150, 0.2850, 0.25, 0 }; // {Static Gain, FeedForward,
                                                                                     // Proportional Gain, ModuleID for
                                                                                     // Tuning}
    public static final double[] kFrontRightTuningVals = { 0.0150, 0.2850, 0.25, 1 }; // {Static Gain, FeedForward,
                                                                                      // Proportional Gain, ModuleID for
                                                                                      // Tuning}
    public static final double[] kBackLeftTuningVals = { 0.0150, 0.2850, 0.25, 2 }; // {Static Gain, FeedForward,
                                                                                    // Proportional Gain, ModuleID for
                                                                                    // Tuning}
    public static final double[] kBackRightTuningVals = { 0.0150, 0.2850, 0.25, 3 }; // {Static Gain, FeedForward,
                                                                                     // Proportional Gain, ModuleID for
                                                                                     // Tuning}

    // NOTE: 2910 Swerve the wheels are not directly under the center of rotation
    // (Take into consideration when measuring)
    public static final double kWheelBaseWidth = 0.5588 / 2; // Half the center distance in meters between right and left wheels on
                                                         // robot
    public static final double kWheelBaseLength = 0.6446 / 2; // Half the center distance in meters between front and back wheels on
                                                          // robot
    public static final double kWheelBaseDistance = Math.sqrt(kWheelBaseWidth * kWheelBaseWidth + kWheelBaseLength * kWheelBaseLength);  // The distance of each wheel from the center

    // Because the swerve modules poisition does not change, define a constant
    // SwerveDriveKinematics for use throughout the code
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBaseLength, kWheelBaseWidth),  // Front Right
      new Translation2d(kWheelBaseLength, -kWheelBaseWidth),  // Front Left
      new Translation2d(-kWheelBaseLength, kWheelBaseWidth),  // Back Right
      new Translation2d(-kWheelBaseLength, -kWheelBaseWidth));  // Back Left

    public static final double kMaxAcceleration = 3.0;
    public static final double kTestMaxAcceleration = 1.0;
    public static final double kMaxSpeedMetersPerSecond = 3.3; // Maximum Sustainable Drivetrain Speed under Normal
                                                                // Conditions & Battery, Robot will not exceed this
                                                                // speed in closed loop control
    public static final double kTestMaxSpeedMetersPerSecond = 1.1;

    public static final double kMaxAngularSpeed = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    public static final double kMaxAngularAccel = Math.PI; // Maximum Angular Acceleration desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly

    public static final double kInnerDeadband = 0.10; // This value should exceed the maximum value the analog stick may
                                                      // read when not in use (Eliminates "Stick Drift")
    public static final double kOuterDeadband = 0.98; // This value should be lower than the analog stick X or Y reading
                                                      // when aimed at a 45deg angle (Such that X and Y are are
                                                      // maximized simultaneously)
    public static final double kTranslationSlew = 1.55;
    public static final double kRotationSlew = 3.00;

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

    public static final double[] kKeepAnglePID = { 0.500, 0, 0 }; // Defines the PID values for the keep angle PID
  }

  public static final class VisionConstants {

  }

  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.6; // Sets a voltage compensation value ideally 12.6V
    public static final int PCHID = 20;
    public static final int PDHID = 24;
    public static final double kLoopTime = 0.020;
  }
}
