// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202;


/**
  Constants here will apply to all robots.
  Only place things that are not expected to vary from robot to robot.

  Values specific to a given year's robot will be in:
      frc.robot<year>.Constants
      
 */
public final class Constants {
  // nominal system frame rate, DT (delta T)
  public static final double DT = 0.02;       // [s] 20ms framerate 50Hz

  // Handy feet to meters
  public static final double FTperM = 3.28084;
  public static final double MperFT = 1.0 / FTperM;
  public static final double DEGperRAD = 57.3;
  public static final int NEO_COUNTS_PER_REVOLUTION = 42;

}