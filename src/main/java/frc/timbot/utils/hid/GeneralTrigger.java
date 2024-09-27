package frc.timbot.utils.hid;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GeneralTrigger extends Trigger {
    private final BooleanSupplier trigger;

    /**
     * Create a joystick trigger for triggering commands.
     *
     * @param joystick   The GenericHID object that has the button (e.g. Joystick,
     *                   KinectStick, etc)
     * @param axisNumber The axis number (see {@link GenericHID#getRawAxis(int) }
     * 
     * @param threshold The value necessary for the trigger to activate (0-1) }     * 
     */
    public GeneralTrigger(BooleanSupplier trigger) {
        this.trigger = trigger;
    }

    @Override
    public boolean get() {
        return trigger.getAsBoolean();
    }
}