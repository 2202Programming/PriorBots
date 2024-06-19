// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.builder;

import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;

/**
 * InnerIRobotSpec
 */
public interface IRobotSpec {

    public String getRobotName();
    public RobotLimits  getRobotLimits();

    //Sensors needed for drivetrain
    IHeadingProvider getHeadingProvider();

    //swerve specs
    public boolean isSwerve();
    public ChassisConfig getChassisConfig();
    public ModuleConfig[] getModuleConfigs();

    public SubsystemConfig getSubsystemConfig();
    public abstract void setBindings();
    public abstract SubsystemConfig[] getSubsystemConfigs();  // some years may have multiple bots
    
    public boolean burnFlash();

}
