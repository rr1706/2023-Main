package frc.robot.utilities;

public class MotionControlState {
    public double m_arm;
    public double m_cube;
    public double m_elevator;
    public double m_wrist;
    public double m_cone;

    public MotionControlState(double arm, double cube, double elevator, double wrist, double cone){
        m_arm = arm;
        m_cube = cube;
        m_elevator = elevator;
        m_wrist = wrist;
        m_cone = cone;
    }
    public MotionControlState(MotionControlState state){
        m_arm = state.m_arm;
        m_cube = state.m_cube;
        m_elevator = state.m_elevator;
        m_wrist = state.m_wrist;
        m_cone = state.m_cone; 
    }

    public void setState(MotionControlState state){
        m_arm = state.m_arm;
        m_cube = state.m_cube;
        m_elevator = state.m_elevator;
        m_wrist = state.m_wrist;
        m_cone = state.m_cone; 
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
    public void setCone(double state) {
        m_cube = state;
    }
}
