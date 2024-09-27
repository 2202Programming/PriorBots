// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chadbot.commands.Shoot;

// Used to hook in a command or other object that tells us we can shoot
// override this based on your targeting system
public interface SolutionProvider {
        default public boolean isOnTarget() { return true;}
}
