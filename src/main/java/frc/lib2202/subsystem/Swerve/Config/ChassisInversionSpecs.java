package frc.lib2202.subsystem.swerve.config;

// Support for multiple robots on same code base
public class ChassisInversionSpecs {
    public ModuleInversionSpecs FR;
    public ModuleInversionSpecs FL;
    public ModuleInversionSpecs BR;
    public ModuleInversionSpecs BL;

    public ChassisInversionSpecs(ModuleInversionSpecs FR, ModuleInversionSpecs FL, ModuleInversionSpecs BR,
            ModuleInversionSpecs BL) {
        this.FR = FR;
        this.FL = FL;
        this.BR = BR;
        this.BL = BL;
    }
}