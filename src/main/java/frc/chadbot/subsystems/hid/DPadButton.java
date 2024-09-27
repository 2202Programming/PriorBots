// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chadbot.subsystems.hid;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

public class DPadButton extends Button {

    XboxController joystick;
    Direction direction;

    public DPadButton(XboxController joystick, Direction direction) {
        this.joystick = joystick;
        this.direction = direction;
    }

    public static enum Direction {
        UP(0), RIGHT(90), DOWN(180), LEFT(270);

        int direction;

        private Direction(int direction) {
            this.direction = direction;
        }
    }

    public boolean get() {
        int dPadValue = joystick.getPOV();
        return (dPadValue == direction.direction);
    }

}
