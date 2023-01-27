package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.MathUtils;

public class DriveByController extends CommandBase {
    private final Drivetrain m_drive;
    private final XboxController m_controller;

    private final SlewRateLimiter m_slewX = new SlewRateLimiter(DriveConstants.kTranslationSlew);
    private final SlewRateLimiter m_slewY = new SlewRateLimiter(DriveConstants.kTranslationSlew);
    private final SlewRateLimiter m_slewRot = new SlewRateLimiter(DriveConstants.kRotationSlew);

    private boolean fieldOrient = true;

    public DriveByController(Drivetrain drive, XboxController controller) {
        m_drive = drive;
        m_controller = controller;
        addRequirements(m_drive);
    }

    @Override
  public void execute() {
    m_drive.drive(m_slewX.calculate(
      -inputTransform(m_controller.getLeftY()))
      * DriveConstants.kMaxSpeedMetersPerSecond,
    m_slewY.calculate(
      -inputTransform(m_controller.getLeftX()))
      * DriveConstants.kMaxSpeedMetersPerSecond,
    m_slewRot.calculate(-inputTransform(m_controller.getRightX()))
      * DriveConstants.kMaxAngularSpeed,
    fieldOrient);
  }

  private double inputTransform(double input) {
    return MathUtils.singedSquare(MathUtils.applyDeadband(input));
  }

}
