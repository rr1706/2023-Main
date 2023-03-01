package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
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

    private final XboxController m_controller;
    private final GenericHID m_operatorBoard;
    private final MotionControlSystem m_controlSystem;

    private final Limelight m_vision;

    private boolean fieldOrient = true;

    private boolean visionLock = false;
    private boolean gyroLock = false;
    private MotionControlState m_state = new MotionControlState(StateConstants.kHome);
    private MotionControlState m_lastState = new MotionControlState(StateConstants.kHome);
    private final PIDController m_rotPID 
    = new PIDController(0.05, 0.002, 0.00);

    public AutoAlign(Drivetrain drive,MotionControlSystem motionSystem, XboxController controller, GenericHID operatorBoard, Limelight vision){
        m_drive = drive;
        m_controller = controller;
        m_operatorBoard = operatorBoard;
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
      m_vision.setLights(2);
      visionLock = false;
      gyroLock = false;
    }

    @Override
    public void execute() {
  
      double maxLinear = DriveConstants.kMaxSpeedMetersPerSecond;
      double desiredX = -inputTransform(1.0*m_controller.getLeftY())*maxLinear;
      double desiredY = -inputTransform(m_controller.getLeftX())*maxLinear;
      Translation2d desiredTranslation = new Translation2d(desiredX, desiredY);
      double desiredMag = desiredTranslation.getDistance(new Translation2d());

      boolean low = m_operatorBoard.getRawButton(4) || m_operatorBoard.getRawButton(5) || m_operatorBoard.getRawButton(6);
      boolean coneMid = m_operatorBoard.getRawButton(7) || m_operatorBoard.getRawButton(9);
      boolean coneHigh = m_operatorBoard.getRawButton(10) || m_operatorBoard.getRawButton(12);
      boolean cubeMid = m_operatorBoard.getRawButton(8);
      boolean cubeHigh = m_operatorBoard.getRawButton(11); 

      if(low){
        m_state = StateConstants.kLow;
        visionLock = false;
        gyroLock = true;
      }
      else if(coneMid){
        m_state = StateConstants.kConeMid;
        visionLock = true;
        gyroLock = false;
        m_vision.setPipeline(2);
      }
      else if(coneHigh){
        m_state = StateConstants.kConeHigh;
        visionLock = true;
        gyroLock = false;
        m_vision.setPipeline(1);
      }
      else if(cubeMid){
        visionLock = false;
        gyroLock = true;
        m_state = StateConstants.kCubeMid;
      }
      else if(cubeHigh){
        visionLock = false;
        gyroLock = true;
        m_state = StateConstants.kCubeHigh;
      }
      else{
        visionLock = false;
        gyroLock = true;
        m_state = StateConstants.kHome;
      }

      if(!m_state.equals(m_lastState)){
        m_controlSystem.setState(m_state);
        m_lastState = m_state;
        SmartDashboard.putBoolean("State Change", true);
      }
      SmartDashboard.putBoolean("State Change", false);

      double desiredRot = 0.0;// m_rotPID.calculate(m_drive.getGyro().getDegrees(),0.0);
      
      if(m_controlSystem.atSetpoint() && visionLock){
        desiredRot = m_rotPID.calculate(m_vision.getTX()+2.0,0.0);
      }
      else if(gyroLock){
        desiredRot = m_rotPID.calculate(m_drive.getGyro().getDegrees(), 180.0);
      }
      else{
        desiredRot = -inputTransform(m_controller.getRightX())* DriveConstants.kMaxAngularSpeed;
      }

      if(desiredMag >= maxLinear){
        desiredTranslation = desiredTranslation.times(maxLinear/desiredMag);
      }
  
      //Translation2d rotAdj= desiredTranslation.rotateBy(new Rotation2d(-Math.PI/2.0)).times(desiredRot*0.05);
  
      //desiredTranslation = desiredTranslation.plus(rotAdj);
  
      m_drive.drive(desiredTranslation.getX(), desiredTranslation.getY(),desiredRot,true,true);
  
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
      m_vision.setLights(1);
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