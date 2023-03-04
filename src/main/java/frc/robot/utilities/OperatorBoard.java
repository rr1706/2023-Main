package frc.robot.utilities;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.FieldConstants;

public class OperatorBoard {

    public static int selectedHeight(GenericHID board) {
        if (validButtons(board)) {
            for (int i = 4; i <= 6; i++) {
                if (board.getRawButton(i)) {return 1;}
            }
            for (int i = 7; i <= 9; i++) {
                if (board.getRawButton(i)) {return 2;}
            }
            for (int i = 10; i <= 12; i++) {
                if (board.getRawButton(i)) {return 3;}
            }
        }
        return -1;
    }

    public static double selectedPosition(GenericHID board) {
        int grid = 0;
        if (validButtons(board)) {
            for (int i = 1; i <= 3; i++) {
                if (board.getRawButton(i)) {grid = i - 1;}
            }
            for (int i = 4; i <= 12; i++) {
                if (board.getRawButton(i)) {return Math.floor((i - 1) / 3) + (grid * 3);}
            }
        }
        return -1.0;
    }

    private static boolean validButtons(GenericHID board) {
        int pressedPositionButtons = 0;
        int pressedHeightButtons = 0;
        for (int i = 0; i <= 2; i++) {
            if (board.getRawButton(i)) {pressedHeightButtons++;}
        }
        for (int i = 3; i <= 12; i++) {
            if (board.getRawButton(i)) {pressedPositionButtons++;}
        }
        return pressedPositionButtons == 1 && pressedHeightButtons == 1;
    }

    public static int scorePosition(GenericHID board){
        if(board.getRawButton(4) || board.getRawButton(5) || board.getRawButton(6)){return 1;}
        else if(board.getRawButton(7) || board.getRawButton(9)){return 2;}
        else if(board.getRawButton(10) || board.getRawButton(12)){return 3;}
        else if(board.getRawButton(8)){return 4;}
        else if(board.getRawButton(11)){return 5;}
        else{return 0;}
    } 

}
