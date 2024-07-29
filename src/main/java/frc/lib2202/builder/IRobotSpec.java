// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.builder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;

/**
 * InnerIRobotSpec
 */
public interface IRobotSpec {

    // basic robot speeds
    public  RobotLimits  getRobotLimits();

    // Sensors needed for drivetrain
    IHeadingProvider getHeadingProvider();

    // swerve specs
    public ChassisConfig getChassisConfig();
    public ModuleConfig[] getModuleConfigs();

    // bindings
    public abstract void setBindings();

    public boolean burnFlash();

    // Setup registered commands
    public SendableChooser<Command> getRegisteredCommands();

    public abstract void setDefaultCommands();

}
