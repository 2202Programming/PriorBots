package frc.robot2019.commands.util ;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DPadButton extends Trigger {

    public DPadButton(XboxController joystick, Direction direction) {
        super( () -> {
            int dPadValue = joystick.getPOV();
            return (dPadValue == direction.direction) || (dPadValue == (direction.direction + 45) % 360)
                    || (dPadValue == (direction.direction + 315) % 360);
        });
    }

    public static enum Direction {
        UP(0), RIGHT(90), DOWN(180), LEFT(270);
        int direction;

        private Direction(int direction) {
            this.direction = direction;
        }
    }
/* was - moved to super() for port to 2025 libs
    public boolean get() {
        int dPadValue = joystick.getPOV();
        return (dPadValue == direction.direction) || (dPadValue == (direction.direction + 45) % 360)
                || (dPadValue == (direction.direction + 315) % 360);
    }
*/
}