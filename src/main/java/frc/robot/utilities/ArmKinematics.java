package frc.robot.utilities;

import edu.wpi.first.math.geometry.Translation2d;

public class ArmKinematics {
    
    public ArmKinematics() {

    }

    public static Translation2d[] reach(Translation2d head, Translation2d tail, Translation2d target) {
        double currentLength = tail.getDistance(head);
        double stretchX = tail.getX() - target.getX();
        double stretchY = tail.getY() - target.getY();
        double stretchLength = tail.getDistance(target);
        double scale = currentLength / stretchLength;

        return new Translation2d[] {
            target,
            target.plus(new Translation2d(stretchX, stretchY)).times(scale)
        };
    }

    public static Translation2d[] iterate(Translation2d[] jointPositions, Translation2d endEffectorTarget) {
        Translation2d[] newJointPositions = jointPositions;

        // Forward Reaching
        Translation2d target = endEffectorTarget;
        for (int i = 0; i < jointPositions.length - 1; i++) {
            Translation2d[] r = reach(newJointPositions[i], newJointPositions[i + 1], endEffectorTarget);
            newJointPositions[i] = r[0];
            endEffectorTarget = r[1];
        }
        newJointPositions[jointPositions.length - 1] = target;

        // Backward Reaching (same as above except with the position array reversed)
        target = jointPositions[0];
        for (int i = jointPositions.length - 1; i > 0; i--) {
            Translation2d[] r = reach(newJointPositions[i], newJointPositions[i - 1], target);
            newJointPositions[i] = r[0];
            target = r[1];
        }
        newJointPositions[0] = target;
        

        return newJointPositions;
    }

    // Not finished
    public static Translation2d[] constrainedIterate(Translation2d[] jointPositions, double[] jointConstraints, Translation2d endEffectorTarget, double[] endEffectorConstraints) {
        Translation2d[] newJointPositions = jointPositions;
        Translation2d target = endEffectorTarget;
        for (int i = 0; i < jointPositions.length - 1; i++) {
            boolean validPoint = jointPositions[i].minus(target).getAngle().getRadians() < jointConstraints[2 * i] && jointPositions[i].minus(target).getAngle().getRadians() > jointConstraints[2 * i + 1];
            Translation2d[] r = reach(
                validPoint ? newJointPositions[i] : null, // not finished
                newJointPositions[i + 1],
                endEffectorTarget
            );
            newJointPositions[i] = r[0];
            endEffectorTarget = r[1];
        }
        newJointPositions[jointPositions.length - 1] = target;

        target = jointPositions[0];
        for (int i = jointPositions.length - 1; i > 0; i--) {
            Translation2d[] r = reach(newJointPositions[i], newJointPositions[i - 1], target);
            newJointPositions[i] = r[0];
            target = r[1];
        }
        newJointPositions[0] = new Translation2d(target.getX(), newJointPositions[1].getY()); // Allow elevator movement

        return newJointPositions;
    }

}
