package frc.robot2019.input.triggers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

public class JoystickTrigger extends Trigger {
    
    /**
     * Create a joystick trigger for triggering commands.
     *
     * @param joystick   The GenericHID object that has the button (e.g. Joystick,
     *                   KinectStick, etc)
     * @param axisNumber The axis number (see {@link GenericHID#getRawAxis(int) }
     * 
     * @param threshold The value necessary for the trigger to activate (0-1) }     
     */
    public JoystickTrigger(GenericHID joystick, int axisNumber, double threshold) {
        //super must be called first, so create the bool function as lambda
        super( () -> joystick.getRawAxis(axisNumber) > threshold ); 
        // good habit to use logging of null
        requireNonNullParam(joystick, "joystick", "JoystickTrigger");
    }
}