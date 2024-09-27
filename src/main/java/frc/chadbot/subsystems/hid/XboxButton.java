// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chadbot.subsystems.hid;

/**
 * 
 *  enum for Xbox buttons.
 * 
 * 2020/12/23  DPL  split buttons and axis for strong typing
 * 
 */
  public enum XboxButton {
    A(1), B(2), X(3), Y(4), // main 4 buttons
    LB(5), RB(6), // bumpers
    BACK(7), START(8), // center of controler
    L3(9), R3(10); // stick buttons

    public final int value;

    private XboxButton(int initValue) {
      value = initValue;
    }

    public int getCode() {
      return value;
    }
  }
  
