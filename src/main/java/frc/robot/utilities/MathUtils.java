package frc.robot.utilities;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.DriveConstants;

public class MathUtils {
    
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

    public static double pythagorean(double a, double b) {
      return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
    }

    /** Return an angle normalized within 0 to 2pi */
    public static double normalizeAngle(double angle) {
      return angle - (2 * Math.PI * Math.floor(angle / (2 * Math.PI)));
    }

    /** Return an angle normalized within the same range as the reference angle */
    public static double normalizeAngle(double angle, double reference) {
      return angle + (2 * Math.PI * Math.floor(reference / (2 * Math.PI)));
    }

    public static Translation2d getIntersection() {
      return new Translation2d(); // not finished
    }

}
