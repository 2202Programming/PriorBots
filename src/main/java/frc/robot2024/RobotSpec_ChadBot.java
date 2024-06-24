package frc.robot2024;

import static frc.lib2202.Constants.MperFT;

import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.subsystem.hid.HID_Xbox_Subsystem;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig.CornerID;
import frc.robot2024.subsystems.Sensors.Sensors_Subsystem;

public class RobotSpec_ChadBot implements IRobotSpec {
    // set this true at least once after robot hw stabilizes
    boolean burnFlash = false;

    boolean swerve = true;

    //Robot Speed Limits
    static double maxSpeed = 15.0 *MperFT; // [m/s]
    static double maxRotationRate = 2.0 * Math.PI; // [rad/s]
    static RobotLimits chadLimits = new RobotLimits(maxSpeed, maxRotationRate);

    //Chassis 
    static double kWheelCorrectionFactor = .995;
    static double kSteeringGR = 12.8;
    static double kDriveGR = 8.14;
    static double kWheelDiameter = MperFT * 4.0/12.0;  // [m]

    static final ChassisConfig chadChassisConfig =
     new ChassisConfig(
        MperFT * (21.516 / 12.0) / 2.0,  // X offset
        MperFT * (24.87 / 12) / 2.0,    // Y offset
        kWheelCorrectionFactor, 
        kWheelDiameter,
        kSteeringGR,
        kDriveGR);

    @Override
    public RobotLimits getRobotLimits() {
       return chadLimits;
    }

    @Override
    public IHeadingProvider getHeadingProvider() {
        return RobotContainer.getSubsystem(Sensors_Subsystem.class);
    }

    @Override
    public boolean isSwerve() {
       return true;
    }

    @Override
    public ChassisConfig getChassisConfig() {
        return chadChassisConfig;
    }

    @Override
    public ModuleConfig[] getModuleConfigs() {
        // from original constants
        // WheelOffsets chadBotOffsets = new WheelOffsets(-175.60, -115.40, -162.15, 158.81); //FL BL FR BR
        //   CANModuleConfig swerveBotCAN_FL = new CANModuleConfig(7, 20, 21);
        //   CANModuleConfig swerveBotCAN_FR = new CANModuleConfig(30, 26, 27);
        //   CANModuleConfig swerveBotCAN_BL = new CANModuleConfig(28, 22, 23);
        //   CANModuleConfig swerveBotCAN_BR = new CANModuleConfig(31, 24, 25);
        //   CANConfig chadBotCANConfig = new CANConfig(swerveBotCAN_FL, swerveBotCAN_FR, swerveBotCAN_BL,    swerveBotCAN_BR);
      
       ModuleConfig[] modules = new ModuleConfig[4];
        modules[CornerID.FrontLeft.getIdx()] = new ModuleConfig(CornerID.FrontLeft, 7,20, 21, -175.60);
        modules[CornerID.FrontRight.getIdx()] = new ModuleConfig(CornerID.FrontRight,30, 26, 27, -162.15);
        modules[CornerID.BackLeft.getIdx()] = new ModuleConfig(CornerID.BackLeft,28, 22,23,-115.40);
        modules[CornerID.BackRight.getIdx()] = new ModuleConfig(CornerID.BackRight,31,24,25, 158.81);
        
       return modules;
    }

    @Override
    public void setBindings() {
        HID_Xbox_Subsystem dc = RobotContainer.getSubsystem("DC");
        //pick one of the next two lines
        BindingsCompetition.ConfigueCompetition(dc);    
        //BindingsOther.ConfigureOther(dc);

    }

    @Override
    public boolean burnFlash() {
      return burnFlash;
    }


}
