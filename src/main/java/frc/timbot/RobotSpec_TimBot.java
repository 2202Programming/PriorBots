package frc.timbot;

import static frc.lib2202.Constants.MperFT;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
import frc.lib2202.command.swerve.FieldCentricDrive;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.command.swerve.calibrate.TestRotateVelocity;
import frc.lib2202.subsystem.Limelight;
import frc.lib2202.subsystem.hid.HID_Xbox_Subsystem;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig.CornerID;
import frc.robot2024.subsystems.sensors.Sensors_Subsystem;
import frc.timbot.commands.Shoot;
import frc.timbot.subsystem.FlywheelSubsystem;

//Swerve bot aka Tim specs
public class RobotSpec_TimBot implements IRobotSpec {
    // set this true at least once after robot hw stabilizes
    boolean burnFlash = false;

    boolean swerve = true;

    // Robot Speed Limits
    static double maxSpeed = 15.0 * MperFT; // [m/s]
    static double maxRotationRate = 360.0;  // [deg/s]
    static RobotLimits robotLimits = new RobotLimits(maxSpeed, maxRotationRate);

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
            .add(Sensors_Subsystem.class)
            .add(Limelight.class)
            .add(SwerveDrivetrain.class)
            .add(HID_Xbox_Subsystem.class, "DC", () -> {
                return new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
            })
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
        return RobotContainer.getSubsystem(Sensors_Subsystem.class);
    }

    @Override
    public ChassisConfig getChassisConfig() {
        return chassisConfig;
    }

    @Override
    public ModuleConfig[] getModuleConfigs() {
        // dpl 8/11/2024   posible fix for https://github.com/2202Programming/PriorBots/issues/1 
        //                 offset angles adjusted along with invert flags.
        //
        // From original 2024 Constants.java:
        //
        // WheelOffsets swerveBotOffsets = new WheelOffsets(-98.942, 91.33, -177.035, -28.215); //FL BL FR BR
        // swerveBotChassisInversionSpecs = new ChassisInversionSpecs(
        //      new ModuleInversionSpecs(true, false, false), // FR  (drvMtrInver, angMtrInvert, angCmdInvert)
        //      new ModuleInversionSpecs(false, false, false), // FL
        //      new ModuleInversionSpecs(true, false, false), // BR
        //      new ModuleInversionSpecs(false, false, false)); // BL
        // CANModuleConfig swerveBotCAN_FL = new CANModuleConfig(7, 20, 21);
        // CANModuleConfig swerveBotCAN_FR = new CANModuleConfig(30, 26, 27);
        // CANModuleConfig swerveBotCAN_BL = new CANModuleConfig(28, 22, 23);
        // CANModuleConfig swerveBotCAN_BR = new CANModuleConfig(31, 24, 25);
       
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
        HID_Xbox_Subsystem dc = RobotContainer.getSubsystem("DC");
        
        var driver = dc.Driver();

        driver.a().onTrue(new Shoot(1000.0));
    }

    @Override
    public boolean burnFlash() {
        return burnFlash;
    }

    @Override
    public SendableChooser<Command> getRegisteredCommands() {
        return null;
    }

    @Override
    public void setDefaultCommands() {
        //Either mode is good for default, debugging easier with RCD.
        //Driving easier with FCD
        @SuppressWarnings("unused")
        Command FCD = new FieldCentricDrive();
        Command RCD = new RobotCentricDrive();
        @SuppressWarnings("unused")
        Command testRotateCMD =  new TestRotateVelocity(60.0, 4.0);
        
        var cmd = RCD;
        SwerveDrivetrain drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
        if (drivetrain != null) {
            drivetrain.setDefaultCommand(cmd);
          }
    }
}
