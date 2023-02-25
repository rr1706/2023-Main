package frc.robot.utilities;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;

public class OperatorBoard {

    public static int selectedHeight(GenericHID board) {
        if (validButtons(board)) {
            for (int i = 1; i <= 3; i++) {
                if (board.getRawButton(i)) {return i;}
            }
        }
        return -1;
    }

    public static int selectedPosition(GenericHID board) {
        if (validButtons(board)) {
            for (int i = 4; i <= 12; i++) {
                if (board.getRawButton(i)) {return i - 3;}
            }
        }
        return -1;
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

}
