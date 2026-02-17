package frc.timbot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static frc.lib2202.Constants.MperFT;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
import frc.lib2202.command.swerve.FieldCentricDrive;
import frc.lib2202.command.swerve.RobotCentricDrive;
//import frc.lib2202.command.swerve.calibrate.TestRotateVelocity;
import frc.lib2202.subsystem.Limelight;
import frc.lib2202.subsystem.Sensors;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig.CornerID;

import frc.timbot.commands.Shoot;
import frc.timbot.subsystem.ShooterLifter;
import frc.timbot.subsystem.FlywheelSubsystem;


//Swerve bot aka Tim specs
public class RobotSpec_TimBot implements IRobotSpec {
    // set this true at least once after robot hw stabilizes
    boolean burnFlash = false;

    boolean swerve = true;

    // Robot Speed Limits
    RobotLimits robotLimits = new RobotLimits(FeetPerSecond.of(15.0), DegreesPerSecond.of(180.0));
    // Chassis
    static double kWheelCorrectionFactor = .995;
    static double kSteeringGR = 12.8;
    static double kDriveGR = 8.14;
    static double kWheelDiameter = MperFT * 4.0 / 12.0; // [m]

    static final ChassisConfig chassisConfig = new ChassisConfig(
            0.62 / 2.0, // X offset [m]
            0.545 / 2.0, // Y offset [m]
            kWheelCorrectionFactor,
            kWheelDiameter,
            kSteeringGR,
            kDriveGR);

    // Subsystems and hardware on Tim 2.0
    SubsystemConfig ssConfig = new SubsystemConfig("SwerveBot - aka Tim", "031b7511")
            .addAlias(Sensors.class, "sensors")
            .add(Limelight.class)
            .addAlias(SwerveDrivetrain.class,"drivetrain")
            //.add(VisionPoseEstimator.class)  //TODO - restore when VPE added to 2202 lib, part of 2025 robot now.
            .add(HID_Subsystem.class, "DC", () -> {
                return new HID_Subsystem(0.3, 0.9, 0.05);
            })
            .add(ShooterLifter.class)
            .add(FlywheelSubsystem.class);

    public RobotSpec_TimBot() {
        ssConfig.setRobotSpec(this);
    }

    // Required methods that use the specs above

    @Override
    public RobotLimits getRobotLimits() {
        return robotLimits;
    }

    @Override
    public IHeadingProvider getHeadingProvider() {
        return RobotContainer.getSubsystem(Sensors.class);
    }

    @Override
    public ChassisConfig getChassisConfig() {
        return chassisConfig;
    }

    @Override
    public ModuleConfig[] getModuleConfigs() {
        ModuleConfig[] modules = new ModuleConfig[4];
        modules[CornerID.FrontLeft.getIdx()] = new ModuleConfig(CornerID.FrontLeft,
                7, 20, 21, -98.942)
                .setInversions(false, false, false);

        modules[CornerID.FrontRight.getIdx()] = new ModuleConfig(CornerID.FrontRight,
                30, 26, 27, -177.035)                
                .setInversions(true, false, false);

        modules[CornerID.BackLeft.getIdx()] = new ModuleConfig(CornerID.BackLeft,
                28, 22, 23,  91.33) 
                .setInversions(false, false, false);

        modules[CornerID.BackRight.getIdx()] = new ModuleConfig(CornerID.BackRight,
                31, 24, 25, -28.215)
                .setInversions(true, false, false);

        return modules;
    }

    @Override
    public void setBindings() {
        HID_Subsystem dc = RobotContainer.getSubsystem("DC");
        
        var driver = dc.Driver();
        if (driver instanceof  CommandXboxController) {
            CommandXboxController xbox_driver = (CommandXboxController)driver;
            xbox_driver.a().onTrue(new Shoot(1000.0));
        }
        else {
            DriverStation.reportError("Timbot expects xbox controller, no driver bindings set, check controllers.", false);
        }
    }

    
    @Override
    public void setDefaultCommands() {
        //Either mode is good for default, debugging easier with RCD.
        //Driving easier with FCD
        @SuppressWarnings("unused")
        Command FCD = new FieldCentricDrive();
        Command RCD = new RobotCentricDrive();
        @SuppressWarnings("unused")
        //Command testRotateCMD =  new TestRotateVelocity(60.0, 4.0);
        
        var cmd = RCD;
        SwerveDrivetrain drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
        if (drivetrain != null) {
            drivetrain.setDefaultCommand(cmd);
          }
    }
}
