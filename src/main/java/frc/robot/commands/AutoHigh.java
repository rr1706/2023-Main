package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.StateConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MotionControlSystem;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.MotionControlState;

public class AutoHigh extends CommandBase {
    private final Drivetrain m_drive;

    private final XboxController m_controller;
    private final GenericHID m_operatorBoard;
    private final MotionControlSystem m_controlSystem;
    private final Claw m_claw;

    private final Limelight m_vision;

    private boolean fieldOrient = true;
    private boolean useLockedPosition = false;
    private final int lockedPosition;
    private final Timer m_timer = new Timer();

    private final InterpolatingTreeMap<Double,Double> m_table = new InterpolatingTreeMap<>();


    private boolean visionLock = false;
    private boolean gyroLock = false;
    private boolean gyroLock180 = false;
    private boolean timeForCube = false;
    private MotionControlState m_state = new MotionControlState(StateConstants.kHome);
    private MotionControlState m_lastState = new MotionControlState(StateConstants.kHome);
    private final PIDController m_rotPID = new PIDController(0.15, 0.00, 0.00);

    private final double m_autoSpeed;

    public AutoHigh(Drivetrain drive,MotionControlSystem motionSystem, XboxController controller, GenericHID operatorBoard, Limelight vision, Claw claw){
        m_drive = drive;
        m_controller = controller;
        m_operatorBoard = operatorBoard;
        m_controlSystem = motionSystem;
        m_vision = vision;
        lockedPosition = -1;
        m_rotPID.enableContinuousInput(-180, 180);
        m_rotPID.setIntegratorRange(-0.2, 0.2);
        m_autoSpeed = 0.0;
        m_claw = claw;

        m_table.put(52.0, 2750.0);
        m_table.put(56.5, 2950.0);
        m_table.put(63.0, 3400.0);

        addRequirements(m_drive);
    }
    public AutoHigh(Drivetrain drive,MotionControlSystem motionSystem, XboxController controller, GenericHID operatorBoard, Limelight vision, Claw claw, int scorePosition, double autoSpeed){
        m_drive = drive;
        m_controller = controller;
        m_operatorBoard = operatorBoard;
        useLockedPosition = true;
        m_autoSpeed = autoSpeed;
        lockedPosition = scorePosition;
        m_controlSystem = motionSystem;
        m_vision = vision;
        m_claw = claw;
        m_rotPID.enableContinuousInput(-180, 180);
        m_rotPID.setIntegratorRange(-0.01, 0.01);

        m_table.put(52.0, 2750.0);
        m_table.put(56.5, 2900.0);
        m_table.put(63.0, 3350.0);

        addRequirements(m_drive);
    }
    
    @Override
    public void initialize(){
      m_state = StateConstants.kHome;
      m_lastState = StateConstants.kHome;
      m_vision.setLights(0);
      visionLock = false;
      gyroLock = false;
      timeForCube = false;
      m_timer.reset();
      m_timer.start();
      if(Math.abs(MathUtil.inputModulus(m_drive.getGyro().getDegrees(),-180,180)) <= 45.0){
        gyroLock180 = true;
      }
      else{
        gyroLock180 = false;
      }
    }

    @Override
    public void execute() {
  
      double maxLinear = DriveConstants.kMaxSpeedMetersPerSecond*0.5;
      double desiredX = -inputTransform(1.0*m_controller.getLeftY())*maxLinear;
      double desiredY = -inputTransform(m_controller.getLeftX())*maxLinear;

      if(useLockedPosition){
        desiredX = -m_autoSpeed;
        desiredY = 0.0;
      }

      Translation2d desiredTranslation = new Translation2d(desiredX, desiredY);
      double desiredMag = desiredTranslation.getDistance(new Translation2d());

        m_state = StateConstants.kConeHigh;
        visionLock = true;
        m_vision.setPipeline(2);

      if(!m_state.equals(m_lastState)){
        m_controlSystem.setState(m_state);
        m_lastState = m_state;
        SmartDashboard.putBoolean("State Change", true);
      }
      SmartDashboard.putBoolean("State Change", false);

      double desiredRot = 0.0;
      
      if(!m_controlSystem.atSetpoint()){
        m_controller.setRumble(RumbleType.kBothRumble, 0.0);
      }

      if(m_controlSystem.atSetpoint() && visionLock && m_vision.valid()){
        double angle = m_vision.getAlign();
        SmartDashboard.putNumber("Angle Error", angle);
        double atAngle = 0.25;

        SmartDashboard.putBoolean("rotatingViaVision", true);
        desiredRot = m_rotPID.calculate(angle,0.0);

        double speedX = m_drive.getChassisSpeed().vxMetersPerSecond;
        double accelX = m_drive.getChassisAccel().ax;

        double speedY = m_drive.getChassisSpeed().vyMetersPerSecond;
        double accelY = m_drive.getChassisAccel().ay;

        speedX = (speedX+accelX*0.024)*39.37;
        speedY = (speedY+accelY*0.024)*39.37;

        double virtualDist = -(speedX*0.460)+m_vision.getDist();

        SmartDashboard.putNumber("Virtual Dist", virtualDist);

        if(m_controller.getRightTriggerAxis() > 0.25 && virtualDist <= 60.0 && speedX >= 0.0 && Math.abs(angle) <= atAngle && Math.abs(speedY) <= 2.0){
          m_claw.setSpeed(m_table.get(virtualDist));
        }


      }
      else if(!m_vision.valid()){
        m_claw.setSpeed(-500);
      }

      if(desiredMag >= maxLinear){
        desiredTranslation = desiredTranslation.times(maxLinear/desiredMag);
      }
  
      if(Math.abs(desiredRot) > 2.0){
        desiredRot = Math.signum(desiredRot)*2.0;
      }

      desiredRot += Math.signum(desiredRot)*Math.abs(m_drive.getChassisSpeed().vxMetersPerSecond)/45.0;

      //Translation2d rotAdj= desiredTranslation.rotateBy(new Rotation2d(-Math.PI/2.0)).times(desiredRot*0.05);
  
      //desiredTranslation = desiredTranslation.plus(rotAdj);
      SmartDashboard.putNumber("AutoAlignROT", desiredRot);
      m_drive.drive(desiredTranslation.getX(), desiredTranslation.getY(),desiredRot,true,false);
  
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
      m_controlSystem.setState(StateConstants.kHome);
      m_vision.setLights(0);
      m_vision.setPipeline(0);
      m_controller.setRumble(RumbleType.kBothRumble, 0.0);
      m_claw.stop();
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