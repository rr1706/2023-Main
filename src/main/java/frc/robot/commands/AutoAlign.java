package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.MotionControlState;

public class AutoAlign extends CommandBase {
    private final Drivetrain m_drive;
    private final Limelight m_vision;

    private final XboxController m_controller;
    private final GenericHID m_operatorBoard;
    private final MotionControlSystem m_controlSystem;

    private boolean fieldOrient = true;
    private boolean useLockedPosition = false;
    private final int lockedPosition;
    private final Timer m_timer = new Timer();

    private boolean visionLock = false;
    private boolean gyroLock = false;
    private boolean gyroLock180 = false;
    private boolean timeForCube = false;
    private MotionControlState m_state = new MotionControlState(StateConstants.kHome);
    private MotionControlState m_lastState = new MotionControlState(StateConstants.kHome);
    private final PIDController m_rotPID = new PIDController(0.1, 0.04, 0.00);

    private final double m_autoSpeed;

    public AutoAlign(Drivetrain drive,MotionControlSystem motionSystem, XboxController controller, GenericHID operatorBoard, Limelight vision){
        m_drive = drive;
        m_controller = controller;
        m_operatorBoard = operatorBoard;
        m_controlSystem = motionSystem;
        m_vision = vision;
        lockedPosition = -1;
        m_rotPID.enableContinuousInput(-180, 180);
        m_rotPID.setIntegratorRange(-0.2, 0.2);
        m_autoSpeed = 0.0;
        addRequirements(m_drive);
    }

    public AutoAlign(Drivetrain drive,MotionControlSystem motionSystem, XboxController controller, GenericHID operatorBoard, Limelight vision, int scorePosition, double autoSpeed){
        m_drive = drive;
        m_controller = controller;
        m_operatorBoard = operatorBoard;
        useLockedPosition = true;
        m_autoSpeed = autoSpeed;
        lockedPosition = scorePosition;
        m_controlSystem = motionSystem;
        m_vision = vision;
        m_rotPID.enableContinuousInput(-180, 180);
        m_rotPID.setIntegratorRange(-0.01, 0.01);
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

    
      boolean low = false;//m_operatorBoard.getRawButton(4) || m_operatorBoard.getRawButton(5) || m_operatorBoard.getRawButton(6);
      boolean coneMid = false;//m_operatorBoard.getRawButton(7) || m_operatorBoard.getRawButton(9);
      boolean coneHigh = false;//m_operatorBoard.getRawButton(10) || m_operatorBoard.getRawButton(12);
      boolean cubeMid = false;//m_operatorBoard.getRawButton(8);
      boolean cubeHigh = false;

        if(useLockedPosition){
            switch(lockedPosition){
                case 1: low = true; break;
                case 2: coneMid = true; break;
                case 3: coneHigh = true; break;
                case 4: cubeMid = true; break;
                case 5: cubeHigh = true; break;
                default: break;
            }
        }
        else{
            low = m_operatorBoard.getRawButton(4) || m_operatorBoard.getRawButton(5) || m_operatorBoard.getRawButton(6);
            coneMid = m_operatorBoard.getRawButton(7) || m_operatorBoard.getRawButton(9);
            coneHigh = m_operatorBoard.getRawButton(10) || m_operatorBoard.getRawButton(12);
            cubeMid = m_operatorBoard.getRawButton(8);
            cubeHigh = m_operatorBoard.getRawButton(11); 
        }

      if(low){
        m_state = StateConstants.kHome;
        visionLock = false;
        gyroLock = true;
      }
      else if(coneMid){
        m_state = StateConstants.kConeMid;
        visionLock = true;
        gyroLock = false;
        m_vision.setPipeline(1);
      }
      else if(coneHigh){
        m_state = StateConstants.kConeHigh;
        visionLock = true;
        gyroLock = false;
        m_vision.setPipeline(2);
      }
      else if(cubeMid){
        visionLock = false;
        gyroLock = true;
        double time = m_timer.get();
        if(gyroLock180 && time<0.80){
            m_state = StateConstants.kRevCubeMidInt;
        }
        else if(gyroLock180 && time < 1.20){
            m_state = StateConstants.kRevCubeMidMid;
        }
        else if(gyroLock180){
            m_state = StateConstants.kRevCubeMidFin;
        }
        else{
            m_state = StateConstants.kCubeMid;
        }
      }
      else if(cubeHigh){
        visionLock = false;
        gyroLock = true;
        m_state = StateConstants.kCubeHigh;
      }
      else{
        visionLock = false;
        gyroLock = false;
        m_state = StateConstants.kHome;
      }

      if(!m_state.equals(m_lastState)){
        m_controlSystem.setState(m_state);
        m_lastState = m_state;
        SmartDashboard.putBoolean("State Change", true);
      }
      SmartDashboard.putBoolean("State Change", false);

      double desiredRot = 0.0;// m_rotPID.calculate(m_drive.getGyro().getDegrees(),0.0);
      
      if(!m_controlSystem.atSetpoint()){
        m_controller.setRumble(RumbleType.kBothRumble, 0.0);
      }

      if(m_controlSystem.atSetpoint() && visionLock){
        double angle = m_vision.getTX();
        double atAngle = 0.4;
        if(coneMid){
            atAngle = 0.8;
        }

        if(Math.abs(angle) <= atAngle && m_vision.getTY()<=-1.40){
            m_controller.setRumble(RumbleType.kBothRumble, 1.0);
        }
        else{
            m_controller.setRumble(RumbleType.kBothRumble, 0.0);
        }
        SmartDashboard.putBoolean("rotatingViaVision", true);
        desiredRot = m_rotPID.calculate(angle,0.0);
        // if((-Math.abs(m_drive.getGyro().getDegrees())+180.0)>15.0){
        //     desiredRot = m_rotPID.calculate(m_drive.getGyro().getDegrees(), 180.0);
        // }

      }
      else if(gyroLock && gyroLock180){
        SmartDashboard.putBoolean("rotatingViaVision", false);
        desiredRot = m_rotPID.calculate(m_drive.getGyro().getDegrees(), 0.0);
      }
      else{
        SmartDashboard.putBoolean("rotatingViaVision", false);
        desiredRot = m_rotPID.calculate(m_drive.getGyro().getDegrees(), 180.0);
      }

      if(desiredMag >= maxLinear){
        desiredTranslation = desiredTranslation.times(maxLinear/desiredMag);
      }
  
      if(Math.abs(desiredRot) > 2.0){
        desiredRot = Math.signum(desiredRot)*2.0;
      }

      if(Math.abs(desiredRot) < 0.05){
        desiredRot = 0.0;
      }

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