package frc.robot.utilities;

import frc.robot.Constants.ArmsConstants;

public class MotionControlState {
    public double m_arm;
    public double m_cubeFront;
    public double m_elevator;
    public double m_wrist;
    public double m_cubeBack;
    public double m_x;
    public double m_y;
    public boolean m_elevatorHigh;

    public MotionControlState(double arm, double cubeFront, double elevator, double wrist, double cubeBack) {
        rawToRect(arm, elevator, wrist);

        m_arm = arm;
        m_cubeFront = cubeFront;
        m_elevator = elevator;
        m_wrist = wrist;
        m_cubeBack = cubeBack;
    }
    /**
     * @param x The x position of the center of the last wheels on the wrist when looking from the side
     * @param y The y position of the center of the last wheels on the wrist when looking from the side
     * @param wristAngle The angle of the wrist in radians with 0 being parallel to the ground
     * @param lowerElevator Whether the elevator should be as low or high as possible
     */
    public MotionControlState(double x, double y, double wristAngle, double cubeFront, double cubeBack, boolean elevatorHigh) {
        rectToRaw(x, y, wristAngle, elevatorHigh);

        m_x = x;
        m_y = y;
        m_cubeBack = cubeBack;
        m_cubeFront = cubeFront;
    }
    public MotionControlState(MotionControlState state){
        setState(state);
    }

    public void setState(MotionControlState state){
        m_arm = state.m_arm;
        m_cubeFront = state.m_cubeFront;
        m_elevator = state.m_elevator;
        m_wrist = state.m_wrist;
        m_cubeBack = state.m_cubeBack;
        m_x = state.m_x;
        m_y = state.m_y;
        m_elevatorHigh = state.m_elevatorHigh;
    }

    public void setArm(double state) {
        m_arm = state;
        rawToRect(m_arm, m_elevator, m_wrist);
    }
    public void setCube(double state) {
        m_cubeFront = state;
    }
    public void setElevator(double state) {
        m_elevator = state;
        rawToRect(m_arm, m_elevator, m_wrist);
    }
    public void setElevatorHighLow(boolean high) {
        m_elevatorHigh = high;
        rectToRaw(m_x, m_y, m_wrist, m_elevatorHigh);
    }
    public void setWrist(double state) {
        m_wrist = state;
        rawToRect(m_arm, m_elevator, m_wrist);
    }
    public void setCone(double state) {
        m_cubeBack = state;
    }
    public void setX(double x, boolean elevatorHigh) {
        m_x = x;
        rectToRaw(m_y, m_y, Math.PI - (m_arm * ArmsConstants.kArmToRadians) - (m_wrist * ArmsConstants.kWristToRadians), elevatorHigh);
    }
    public void setY(double y, boolean elevatorHigh) {
        m_y = y;
        rectToRaw(m_y, m_y, Math.PI - (m_arm * ArmsConstants.kArmToRadians) - (m_wrist * ArmsConstants.kWristToRadians), elevatorHigh);
    }

    private void rectToRaw(double x, double y, double wrist, boolean elevatorHigh) {
        double x0 = x - ArmsConstants.kWristLength * Math.cos(wrist);
        double y0 = y - ArmsConstants.kWristLength * Math.sin(wrist);
        double h = y0 + (elevatorHigh ? 1 : -1) * Math.sqrt(Math.pow(ArmsConstants.kArmLength, 2) - Math.pow(x0, 2));
        double a = Math.asin(x0 / ArmsConstants.kArmLength);
        double w = Math.PI / 2 - a - wrist;

        m_x = x;
        m_y = y;
        m_arm = a / ArmsConstants.kArmToRadians;
        m_elevator = h / ArmsConstants.kElevatorToCentimeters;
        m_wrist = w / ArmsConstants.kWristToRadians;

        if (m_elevator < ArmsConstants.kMinElevator || m_elevator > ArmsConstants.kMaxElevator) {
            h = y0 + (!elevatorHigh ? 1 : -1) * Math.sqrt(Math.pow(ArmsConstants.kArmLength, 2) - Math.pow(x0, 2));
            m_elevator = h / ArmsConstants.kElevatorToCentimeters;
        }
    }

    private void rawToRect(double arm, double elevator, double wrist) {
        double w = Math.PI / 2 - (arm * ArmsConstants.kArmToRadians) - (wrist * ArmsConstants.kWristToRadians);
        m_x = ArmsConstants.kArmLength * Math.cos(m_arm * ArmsConstants.kArmToRadians) + (ArmsConstants.kWristLength * Math.cos(w));
        m_y = elevator * ArmsConstants.kElevatorToCentimeters + (ArmsConstants.kArmLength * Math.sin(arm * ArmsConstants.kArmToRadians)) + (ArmsConstants.kWristLength * Math.sin(w));
        m_elevatorHigh = m_y >= ArmsConstants.kWristLength * Math.sin(w);
    }
}
