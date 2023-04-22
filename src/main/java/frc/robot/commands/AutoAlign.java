package frc.robot.commands;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.vision.VisionThread;
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

public class AutoAlign extends CommandBase {
    private final Drivetrain m_drive;

    private final XboxController m_controller;
    private final GenericHID m_operatorBoard;
    private final MotionControlSystem m_controlSystem;
    private final Claw m_claw;

    private final Limelight m_visionTop;
    private final Limelight m_visionBottom;

    private boolean fieldOrient = true;
    private final boolean useLockedPosition;
    private final int lockedPosition;
    private final Timer m_timer = new Timer();

    private boolean visionLock = false;
    private boolean gyroLock = false;
    private boolean gyroLock180 = false;
    private boolean timeForCube = false;
    private MotionControlState m_state = new MotionControlState(StateConstants.kHome);
    private MotionControlState m_lastState = new MotionControlState(StateConstants.kHome);
    private final PIDController m_rotPID = new PIDController(0.145, 0.00, 0.0025);

    private final InterpolatingTreeMap<Double,Double> m_rpmHigh = new InterpolatingTreeMap<>();
    private final InterpolatingTreeMap<Double, Double> m_distHigh = new InterpolatingTreeMap<>();
    private final InterpolatingTreeMap<Double, Double> m_rpmMid = new InterpolatingTreeMap<>();
    private final InterpolatingTreeMap<Double, Double> m_distMid = new InterpolatingTreeMap<>();
    private final InterpolatingTreeMap<Double, Double> m_distMidFromBottom = new InterpolatingTreeMap<>();
    private final InterpolatingTreeMap<Double, Double> m_timeTableMid = new InterpolatingTreeMap<>();
    private final InterpolatingTreeMap<Double, Double> m_timeTableHigh = new InterpolatingTreeMap<>();

    private final double m_autoSpeed;

    // Top and Bottom refers to the position of limelight
    
    public AutoAlign(Drivetrain drive, MotionControlSystem motionSystem, Claw claw, XboxController controller, GenericHID operatorBoard, Limelight limelightTop, Limelight limelightBottom, int scorePosition, double autoSpeed){
      m_drive = drive;
      m_controller = controller;
      m_operatorBoard = operatorBoard;
      useLockedPosition = scorePosition != -1;
      m_autoSpeed = autoSpeed;
      lockedPosition = scorePosition;
      m_controlSystem = motionSystem;
      m_claw = claw;
      m_visionTop = limelightTop;
      m_visionBottom = limelightBottom;
      m_rotPID.enableContinuousInput(-180, 180);
      m_rotPID.setIntegratorRange(-0.02, 0.02);

      m_rpmHigh.put(52.0, 2150.0);
      m_rpmHigh.put(55.5, 2500.0);
      m_rpmHigh.put(56.5, 2550.0);
      m_rpmHigh.put(63.0, 3200.0);

      m_distHigh.put(2.0, 51.0);
      m_distHigh.put(0.0, 58.0);
      m_distHigh.put(-2.0, 63.5);
      m_distHigh.put(-4.0, 69.5);
      m_distHigh.put(-6.0, 77.5);
      m_distHigh.put(-8.0, 86.5);
      m_distHigh.put(-10.0, 100.0);

      m_rpmMid.put(34.0, 1200.0);
      m_rpmMid.put(40.0, 1375.0);
      m_rpmMid.put(46.0, 1900.0);
      m_rpmMid.put(52.0, 2465.0);
      m_rpmMid.put(58.0, 3300.0);

      m_distMidFromBottom.put(1.5, 37.0);
      m_distMidFromBottom.put(0.0, 43.0);
      m_distMidFromBottom.put(-1.5, 50.0);
      m_distMidFromBottom.put(-3.0, 60.0);
      m_distMidFromBottom.put(-4.5, 73.5);
      m_distMidFromBottom.put(-6.0, 93.5);
      m_distMidFromBottom.put(-7.5, 130.0);

      m_distMid.put(-3.03, 30.75);
      m_distMid.put(0.05, 27.5);
      m_distMid.put(2.99, 34.25);
      m_distMid.put(6.00, 38.75);
      m_distMid.put(9.02, 44.0);
      m_distMid.put(12.02, 50.5);
      m_distMid.put(14.99, 58.5);

      m_timeTableHigh.put(52.0, 0.440);
      m_timeTableHigh.put(56.0, 0.465);
      m_timeTableHigh.put(63.0, 0.500);

      m_timeTableMid.put(34.0, 0.200);
      m_timeTableMid.put(40.0, 0.220);
      m_timeTableMid.put(46.0, 0.230);
      m_timeTableMid.put(52.0, 0.240);
      m_timeTableMid.put(58.0, 0.250);

      addRequirements(m_drive);
    }

    public AutoAlign(Drivetrain drive, MotionControlSystem motionSystem, Claw claw, XboxController controller, GenericHID operatorBoard, Limelight limelightTop, Limelight limelightBottom){
      this(
        drive,
        motionSystem,
        claw,
        controller,
        operatorBoard,
        limelightTop,
        limelightBottom,
        -1,
        0.0
      );
    }
    
    @Override
    public void initialize(){
      m_state = StateConstants.kHome;
      m_lastState = StateConstants.kHome;
      m_visionBottom.setLights(0);
      m_visionBottom.setPipeline(0);
      m_visionTop.setLights(0);
      m_visionTop.setPipeline(0);
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
  
      fieldOrient = true;

      double maxLinear = DriveConstants.kMaxSpeedMetersPerSecond*0.2;
      double desiredX = -inputTransform(1.0*m_controller.getLeftY())*maxLinear;
      double desiredY = -inputTransform(m_controller.getLeftX())*maxLinear;

      if(useLockedPosition){
        desiredX = -m_autoSpeed;
        desiredY = 0.0;
      }
    
      boolean low = false;//m_operatorBoard.getRawButton(4) || m_operatorBoard.getRawButton(5) || m_operatorBoard.getRawButton(6);
      boolean coneMid = false;//m_operatorBoard.getRawButton(7) || m_operatorBoard.getRawButton(9);
      boolean coneHigh = false;//m_operatorBoard.getRawButton(10) || m_operatorBoard.getRawButton(12);
      boolean cubeMid = false;//m_operatorBoard.getRawButton(8);
      boolean cubeHigh = false;
      boolean superCharge = false;

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
            superCharge = m_operatorBoard.getRawButton(3);
            low = m_operatorBoard.getRawButton(4) || m_operatorBoard.getRawButton(5) || m_operatorBoard.getRawButton(6);
            coneMid = m_operatorBoard.getRawButton(7) || m_operatorBoard.getRawButton(9);
            coneHigh = m_operatorBoard.getRawButton(10) || m_operatorBoard.getRawButton(12);
            cubeMid = m_operatorBoard.getRawButton(8);
            cubeHigh = m_operatorBoard.getRawButton(11); 
        }

      if(low){
        m_state = StateConstants.kLow;
        visionLock = false;
        gyroLock = true;
      }
      else if(coneMid){
        m_state = StateConstants.kConeMid;
        visionLock = true;
        gyroLock = false;
        m_visionBottom.setLights(3);
        if(superCharge){
          m_visionBottom.setPipeline(3);
        }
        else{
          m_visionBottom.setPipeline(1);
        }
      }
      else if(coneHigh){
        m_state = StateConstants.kConeHigh;
        visionLock = true;
        gyroLock = false;
        if(superCharge){
          m_visionBottom.setPipeline(2);
        }
        else{
          m_visionBottom.setPipeline(0);
        }
        m_visionBottom.setLights(3);
      }
      else if(cubeMid || (cubeHigh && gyroLock180)){
        visionLock = false;
        gyroLock = true;
        double time = m_timer.get();
        if(gyroLock180 && time<0.6){
            m_state = StateConstants.kRevCubeMidInt;
        }
        else if(gyroLock180 && time < 0.8){
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
        m_visionBottom.setLights(1);
        m_visionTop.setPipeline(0);
      }

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

      boolean clearForHigh = m_controlSystem.clearForHigh() && coneHigh;
      boolean clearForMid = m_controlSystem.clearForMid() && coneMid;

      if((m_controlSystem.atSetpoint() || clearForHigh || clearForMid) && visionLock && ((m_visionBottom.valid() && (coneHigh || coneMid)))){
        double angle = coneHigh ? m_visionBottom.getTX()-(Math.toDegrees(Math.asin(8.0/m_distHigh.get(m_visionBottom.getTY())))-8.102) : 
                                  m_visionBottom.getTX()-(Math.toDegrees(Math.asin(8.0/m_distMidFromBottom.get(m_visionBottom.getTY())))-11.535);
        
        SmartDashboard.putNumber("Angle Error", angle);
        double atAngle = 0.25;


        SmartDashboard.putBoolean("rotatingViaVision", true);
        desiredRot = m_rotPID.calculate(angle,0.0);

        double speedX = m_drive.getChassisSpeed().vxMetersPerSecond;
        double accelX = m_drive.getChassisAccel().ax;

        double speedY = m_drive.getChassisSpeed().vyMetersPerSecond;
        double accelY = m_drive.getChassisAccel().ay;

        double speedRot = m_drive.getChassisSpeed().omegaRadiansPerSecond;
        double accelRot = m_drive.getChassisAccel().alpha;

        speedX = (speedX+accelX*0.024)*39.37;
        speedY = (speedY+accelY*0.024)*39.37;
        speedRot = (speedRot+accelRot*0.024);

        double dist = coneHigh ? m_distHigh.get(m_visionBottom.getTY()): m_distMidFromBottom.get(m_visionBottom.getTY());
        double shotTime = coneHigh ? m_timeTableHigh.get(dist): m_timeTableMid.get(dist);
        double virtualDist = -speedX*shotTime+dist;

        for(int i=0;i<5;i++){

          double testDist = -speedX*shotTime+dist;

          double newShotTime = coneHigh ? m_timeTableHigh.get(testDist) : m_timeTableMid.get(testDist);

          if(Math.abs(newShotTime-shotTime) <= 0.010){
            i=4;
        }
        if(i == 4){
          virtualDist = testDist;
          SmartDashboard.putNumber("NewShotTime", newShotTime);
      }
        else{
            shotTime = newShotTime;
        }

        }
        SmartDashboard.putNumber("Virtual Dist", virtualDist);

        if(m_controller.getRightTriggerAxis() > 0.25 && virtualDist <= (coneHigh ? 61.5 : 52.0) && virtualDist >= (coneHigh ? 52.0 : 40.0) && speedX > -6.0 && Math.abs(angle) <= atAngle && Math.abs(speedY) <= 2.0 && Math.abs(speedRot) <= 0.05 && speedX < 18.0 && !superCharge){
          m_claw.setSpeed(coneHigh ? m_rpmHigh.get(virtualDist) : m_rpmMid.get(virtualDist));
        }
        else if(superCharge && m_controller.getRightTriggerAxis() > 0.25 && Math.abs(speedX) <= 2.0 && Math.abs(speedY) <= 2.0){
          m_claw.setSpeed(coneHigh ? 2485.0 : 1350);
        }

        desiredRot += Math.signum(desiredRot)*Math.abs(m_drive.getChassisSpeed().vxMetersPerSecond)/40.0;

        desiredRot *= coneHigh ? 1.0 : 0.8; 

        fieldOrient = false;
        desiredX *= -1.0;
        desiredY *= -0.5;
      }
      else if(gyroLock && gyroLock180){
        SmartDashboard.putBoolean("rotatingViaVision", false);
        desiredRot = m_rotPID.calculate(m_drive.getGyro().getDegrees(), 0.0);
      }
      else{
        SmartDashboard.putBoolean("rotatingViaVision", false);
        desiredRot = m_rotPID.calculate(m_drive.getGyro().getDegrees(), 180.0);
      }
      if(m_controller.getRightTriggerAxis()>0.25 && !(coneHigh || coneMid)){
        if(cubeHigh){
          m_claw.setSpeed(2500);
        }
        else if(cubeMid){
          m_claw.setSpeed(1750);
        }
        else if(low){
          m_claw.setSpeed(1500);
        }
        else{
          m_claw.setSpeed(4500);
        }
      }

      Translation2d desiredTranslation = new Translation2d(desiredX, desiredY);
      double desiredMag = desiredTranslation.getDistance(new Translation2d());

      if(desiredMag >= maxLinear){
        desiredTranslation = desiredTranslation.times(maxLinear/desiredMag);
      }
  
      if(Math.abs(desiredRot) > 2.0){
        desiredRot = Math.signum(desiredRot)*2.0;
      }

      //Translation2d rotAdj= desiredTranslation.rotateBy(new Rotation2d(-Math.PI/2.0)).times(desiredRot*0.05);
  
      //desiredTranslation = desiredTranslation.plus(rotAdj);
      SmartDashboard.putNumber("AutoAlignROT", desiredRot);
      m_drive.drive(desiredTranslation.getX(), desiredTranslation.getY(),desiredRot,fieldOrient,false);
  
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
      m_visionTop.setLights(0);
      m_visionBottom.setLights(0);
      m_visionTop.setPipeline(0);
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
