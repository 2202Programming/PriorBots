// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.builder;

import frc.lib2202.subsystem.swerve.config.CANConfig;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ChassisInversionSpecs;
import frc.lib2202.subsystem.swerve.config.WheelOffsets;

/**
 * InnerIRobotSpec
 */
public interface IRobotSpec {

    public String getRobotName();
    public RobotLimits  getRobotLimits();

    //swerve specs
    public boolean isSwerve();
    public WheelOffsets getWheelOffset();
    public ChassisConfig getChassisConfig();
    public SubsystemConfig getSubsystemConfig();
    public ChassisInversionSpecs getChassisInversionSpecs();
    public CANConfig getCANConfig();

    public abstract void setBindings();
    public abstract SubsystemConfig[] getSubsystemConfigs();  // some years may have multiple bots
    
    public boolean burnFlash();

}
