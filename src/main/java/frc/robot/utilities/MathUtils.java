package frc.robot.utilities;

import frc.robot.Constants.DriveConstants;

public class MathUtils {
    
    public static double toUnitCircAngle(double angle) {
      double rotations = angle / (2 * Math.PI);
      return (angle - Math.round(rotations - 0.500) * Math.PI * 2.0);
    }

    public static double singedSquare(double input) {
      return Math.signum(input) * Math.pow(input, 2);
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

}
