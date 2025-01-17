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
    public abstract RobotLimits  getRobotLimits();

    // Sensors needed for drivetrain
    public abstract IHeadingProvider getHeadingProvider();

    // swerve specs
    public abstract ChassisConfig getChassisConfig();
    public abstract ModuleConfig[] getModuleConfigs();

    // bindings
    public abstract void setBindings();

    default public boolean burnFlash(){ return true;};

    // Setup registered commands
    public abstract SendableChooser<Command> getRegisteredCommands();

    public abstract void setDefaultCommands();

    default public void teleopInit(){}

}
