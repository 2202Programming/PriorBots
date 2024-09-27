// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chadbot.subsystems.ifx;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface MecanumDriveIfx extends Subsystem {
    void drive_normalized (double x, double y, double rotation);
}