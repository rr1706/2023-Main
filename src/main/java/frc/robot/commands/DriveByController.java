package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.MathUtils;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Implements a DriveByController command which extends the CommandBase class
 */
public class DriveByController extends CommandBase {
  private final Drivetrain m_robotDrive;
  private final XboxController m_controller;

  private boolean fieldOrient = true;

  /**
   * Contructs a DriveByController object which applys the driver inputs from the
   * controller to the swerve drivetrain
   * 
   * @param drive      is the swerve drivetrain object which should be created in
   *                   the RobotContainer class
   * @param controller is the user input controller object for controlling the
   *                   drivetrain
   */
  public DriveByController(Drivetrain drive, XboxController controller) {
    m_robotDrive = drive; // Set the private member to the input drivetrain
    m_controller = controller; // Set the private member to the input controller
    addRequirements(m_robotDrive); // Because this will be used as a default command, add the subsystem which will
                                   // use this as the default
  }

  /**
   * the execute function is overloaded with the function to drive the swerve
   * drivetrain
   */
  @Override
  public void execute() {

    double maxLinear = DriveConstants.kMaxSpeedMetersPerSecond;
    double desiredX = -inputTransform(1.0*m_controller.getLeftY())*maxLinear;
    double desiredY = -inputTransform(m_controller.getLeftX())*maxLinear;
    Translation2d desiredTranslation = new Translation2d(desiredX, desiredY);
    double desiredMag = desiredTranslation.getDistance(new Translation2d());

    double desiredRot = -inputTransform(m_controller.getRightX())* DriveConstants.kMaxAngularSpeed;


    if(desiredMag >= maxLinear){
      desiredTranslation = desiredTranslation.times(maxLinear/desiredMag);
    }

    //Translation2d rotAdj= desiredTranslation.rotateBy(new Rotation2d(-Math.PI/2.0)).times(desiredRot*0.05);

    //desiredTranslation = desiredTranslation.plus(rotAdj);

    m_robotDrive.drive(desiredTranslation.getX(), desiredTranslation.getY(),desiredRot,true,true);

/*     m_robotDrive.drive(m_slewX.calculate(
        -inputTransform(m_controller.getLeftY()))
        * DriveConstants.kMaxSpeedMetersPerSecond,
        m_slewY.calculate(
            -inputTransform(m_controller.getLeftX()))
            * DriveConstants.kMaxSpeedMetersPerSecond,
        m_slewRot.calculate(-inputTransform(m_controller.getRightX()))
            * DriveConstants.kMaxAngularSpeed,
        fieldOrient); */

        SmartDashboard.putBoolean("DrivingByController", true);
  }

  @Override
  public void end(boolean interrupted){
    SmartDashboard.putBoolean("DrivingByController", false);

  }

  /**
   * when this fucntion of the command is called the current fieldOrient boolean
   * is flipped. This
   * is fed into the drive command for the swerve drivetrain so the driver can
   * decide to drive in
   * a robot oreinted when they please (not recommended in most instances)
   */
  public void changeFieldOrient() {
    if (fieldOrient) {
      fieldOrient = false;
    } else {
      fieldOrient = true;
    }
  }

  private double inputTransform(double input){
    //return MathUtils.singedSquare(MathUtils.applyDeadband(input));
    return MathUtils.cubicLinear(MathUtils.applyDeadband(input), 0.95, 0.05);
  }

}
