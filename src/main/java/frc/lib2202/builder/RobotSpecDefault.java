package frc.lib2202.builder;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static frc.lib2202.Constants.MperFT;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;

//copy or extend this code for your robot - remember to override:
// TBD
//
public class RobotSpecDefault implements IRobotSpec {
    // This config does nothing, no subsystems added but will let any new RoboRio
    // Build

    // Robot Speed Limits
    RobotLimits robotLimits = new RobotLimits(
        FeetPerSecond.of(15.0), 
        DegreesPerSecond.of(360.0));

    // Chassis
    double kWheelCorrectionFactor = .995;
    double kSteeringGR = 12.8;
    double kDriveGR = 8.14;
    double kWheelDiameter = MperFT * 4.0 / 12.0; // [m]

    ChassisConfig chassisConfig = new ChassisConfig(
            MperFT * (21.516 / 12.0) / 2.0, // X offset [m]
            MperFT * (24.87 / 12) / 2.0, // Y offset [m]
            kWheelCorrectionFactor, // []
            kWheelDiameter, // [m]
            kSteeringGR,    // []
            kDriveGR);      // []

    // SubsystemConfig gets registered in static array to match serial number at Construct call
    SubsystemConfig subsystemConfig = new SubsystemConfig("DEFAULT:bot-On-Board", "00000000");

    @Override
    public RobotLimits getRobotLimits() {
        return robotLimits;
    }

    @Override
    public IHeadingProvider getHeadingProvider() {
        return null;  // no sensors in default, example for your bot's Spec:
        //return RobotContainer.getSubsystem(Sensors_Subsystem.class);
    }

    @Override
    public ChassisConfig getChassisConfig() {
        return chassisConfig;
    }

    @Override
    public ModuleConfig[] getModuleConfigs() {
        return null;
    }

    @Override
    public void setBindings() {
        // no bindings
    }

    @Override
    public boolean burnFlash() {
        return false;
    }

    @Override
    public SendableChooser<Command> getRegisteredCommands() {
      return null;
    }

    @Override
    public void setDefaultCommands() {
      
    }


}
