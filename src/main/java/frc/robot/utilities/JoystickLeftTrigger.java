package frc.robot.utilities;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.*;

public class JoystickLeftTrigger extends Trigger {

  /**
   * Create a button for triggering commands off a controller's analog axis
   * 
   * @param controller The controller to use
   * @param side Which side of the controller (Left = true, Right = false)
   */
  public JoystickLeftTrigger(XboxController controller) {
    this(controller, 0.25);
  }

  /**
   * Create a button for triggering commands off a controller's analog axis
   * 
   * @param controller The controller to use
   * @param side Which side of the controller (Left = true, Right = false)
   */
  public JoystickLeftTrigger(XboxController controller, double threshold) {
    super(() -> controller.getLeftTriggerAxis() >= threshold);
  }


}