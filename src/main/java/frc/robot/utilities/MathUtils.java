package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.Constants.DriveConstants;

public class MathUtils {

    private static final double kEps = 1E-9;
    
    public static double toUnitCircAngle(double angle) {
      double rotations = angle / (2 * Math.PI);
      return (angle - Math.round(rotations - 0.500) * Math.PI * 2.0);
    }

    public static double singedSquare(double input) {
      return Math.signum(input) * Math.pow(input, 2);
    }

    public static double cubicLinear(double input, double a, double b){
      return (a*Math.pow(input, 3)+b*input);
    }

    public static double applyDeadband(double input) {
      if (Math.abs(input) < DriveConstants.kInnerDeadband) {
        return 0.0;
      } else if (Math.abs(input) > DriveConstants.kOuterDeadband) {
        return Math.signum(input) * 1.0;
      } else {
        return input;
      }
    }

    public static double inputTransform(double input){
      //return MathUtils.singedSquare(MathUtils.applyDeadband(input));
      return cubicLinear(applyDeadband(input), 0.95, 0.05);
    }

    public static double[] inputTransform(double x, double y){
      x = applyDeadband(x);
      y = applyDeadband(y);
      double mag = pythagorean(x,y);

      if(mag>1.00){
        mag = 1.00;
      }

      if(mag != 0){
        x = x/mag*cubicLinear(mag, 0.95, 0.05);
        y = y/mag*cubicLinear(mag, 0.95, 0.05);  
      }
      else{
        x = 0;
        y = 0;
      }

      return new double[]{x,y};
    }

    public static double pythagorean(double a, double b) {
      return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
    }
    
    public static Twist2d log(final Pose2d transform) {
      final double dtheta = transform.getRotation().getRadians();
      final double half_dtheta = 0.5 * dtheta;
      final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
      double halftheta_by_tan_of_halfdtheta;
      if (Math.abs(cos_minus_one) < kEps) {
        halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
      } else {
        halftheta_by_tan_of_halfdtheta =
            -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;
      }
      final Translation2d translation_part =
          transform
              .getTranslation()
              .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
      return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
  }

}
