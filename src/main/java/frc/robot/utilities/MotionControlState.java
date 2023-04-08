package frc.robot.utilities;

import frc.robot.Constants.ArmsConstants;
import frc.robot.Constants.DriveConstants;

public class MotionControlState {
    public double m_arm;
    public double m_cube;
    public double m_elevator;
    public double m_wrist;
    public double m_cone;
    public double m_x;
    public double m_y;

    private boolean m_lowerElevator;

    public MotionControlState(double arm, double cube, double elevator, double wrist, double cone){
        rawToRect(arm, elevator, wrist);

        m_arm = arm;
        m_cube = cube;
        m_elevator = elevator;
        m_wrist = wrist;
        m_cone = cone;
    }
    /**
     * @param x The x position of the center of the last wheels on the wrist when looking from the side
     * @param y The y position of the center of the last wheels on the wrist when looking from the side
     * @param wristAngle The angle of the wrist in radians with 0 being parallel to the ground
     * @param lowerElevator Whether the elevator should be as low or high as possible
     */
    public MotionControlState(double x, double y, double wristAngle, double cone, double cube, boolean lowerElevator) {
        rectToRaw(x, y, wristAngle, lowerElevator);

        m_x = x;
        m_y = y;
        m_cone = cone;
        m_cube = cube;
    }
    public MotionControlState(MotionControlState state){
        m_arm = state.m_arm;
        m_cube = state.m_cube;
        m_elevator = state.m_elevator;
        m_wrist = state.m_wrist;
        m_cone = state.m_cone;
        m_x = state.m_x;
        m_y = state.m_y;
        m_lowerElevator = state.m_lowerElevator;
    }

    public void setState(MotionControlState state){
        m_arm = state.m_arm;
        m_cube = state.m_cube;
        m_elevator = state.m_elevator;
        m_wrist = state.m_wrist;
        m_cone = state.m_cone;
        m_x = state.m_x;
        m_y = state.m_y;
    }

    public void setArm(double state) {
        m_arm = state;
        rawToRect(m_arm, m_elevator, m_wrist);
    }
    public void setCube(double state) {
        m_cube = state;
    }
    public void setElevator(double state) {
        m_elevator = state;
        rawToRect(m_arm, m_elevator, m_wrist);
    }
    public void setWrist(double state) {
        m_wrist = state;
        rawToRect(m_arm, m_elevator, m_wrist);
    }
    public void setCone(double state) {
        m_cube = state;
    }
    public void setX(double x, boolean lowerElevator) {
        m_x = x;
        rectToRaw(m_y, m_y, Math.PI - (m_arm * ArmsConstants.kArmToRadians) - (m_wrist * ArmsConstants.kWristToRadians), lowerElevator);
    }
    public void setY(double y, boolean lowerElevator) {
        m_y = y;
        rectToRaw(m_y, m_y, Math.PI - (m_arm * ArmsConstants.kArmToRadians) - (m_wrist * ArmsConstants.kWristToRadians), lowerElevator);
    }

    private void rectToRaw(double x, double y, double wristAngle, boolean lowerElevator) {
        double theta = ((Math.pow(x, 2) + Math.pow(y, 2)) + (Math.pow(ArmsConstants.kArmLength, 2) + Math.pow(ArmsConstants.kWristLength, 2))) / (2 * ArmsConstants.kArmLength * ArmsConstants.kWristLength);
        double joint = y - ArmsConstants.kWristLength * Math.sin(wristAngle);
        double elevator = joint;
        double arm;
        double wrist;
        if (elevator / ArmsConstants.kElevatorToCentimeters >= ArmsConstants.kMinElevator || lowerElevator) {
            elevator -= ArmsConstants.kArmLength * Math.sin(wristAngle - theta);
            arm = wristAngle - theta;
            wrist = -theta;
        } else {
            elevator -= ArmsConstants.kArmLength * Math.sin(wristAngle + theta);
            arm = wristAngle + theta;
            wrist = theta + (Math.PI / 2);
        }
        elevator /= ArmsConstants.kElevatorToCentimeters;
        arm /= ArmsConstants.kArmToRadians;
        wrist /= ArmsConstants.kWristToRadians;

        m_elevator = elevator;
        m_arm = arm;
        m_wrist = wrist;
        m_lowerElevator = lowerElevator;
    }

    private void rawToRect(double arm, double elevator, double wrist) {
        m_x = (ArmsConstants.kArmLength * Math.cos(m_arm * ArmsConstants.kArmToRadians)) + (ArmsConstants.kWristLength * Math.cos(Math.PI - (m_arm * ArmsConstants.kArmToRadians) - (m_wrist * ArmsConstants.kWristToRadians)));
        m_y = (elevator * ArmsConstants.kElevatorToCentimeters) + (ArmsConstants.kArmLength * Math.sin(m_arm * ArmsConstants.kArmToRadians)) + (ArmsConstants.kWristLength * Math.sin(Math.PI - (m_arm * ArmsConstants.kArmToRadians) - (m_wrist * ArmsConstants.kWristToRadians)));
    }
}
