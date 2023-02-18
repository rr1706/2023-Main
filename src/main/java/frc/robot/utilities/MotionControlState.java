package frc.robot.utilities;

public class MotionControlState {
    public double m_arm;
    public double m_cube;
    public double m_elevator;
    public double m_wrist;

    public MotionControlState(double arm, double cube, double elevator, double wrist ){
        m_arm = arm;
        m_cube = cube;
        m_elevator = elevator;
        m_wrist = wrist;
    }

    public void setArm(double state) {
        m_arm = state;
    }
    public void setCube(double state) {
        m_cube = state;
    }
    public void setElevator(double state) {
        m_elevator = state;
    }
    public void setWrist(double state) {
        m_wrist = state;
    }
}
