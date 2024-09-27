// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chadbot.subsystems.hid;

/**
 * enum for Xbox analog inputs (Axis)
 * 
 * 2020/12/23 DPL split from combined enum for strong type
 * 
 */
public enum XboxAxis {
    // analog triggers aka Axis
    LEFT_X(0), LEFT_Y(1), TRIGGER_LEFT(2), // left side
    TRIGGER_RIGHT(3), RIGHT_X(4), RIGHT_Y(5); // right side

    public final int value;

    private XboxAxis(int initValue) {
        value = initValue;
    }

    public int getCode() {
        return value;
    }
}
