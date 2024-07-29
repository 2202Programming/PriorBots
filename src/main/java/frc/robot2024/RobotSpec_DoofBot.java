package frc.robot2024;

import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
import frc.lib2202.command.swerve.FieldCentricDrive;
import frc.lib2202.subsystem.Limelight;
import frc.lib2202.subsystem.hid.HID_Xbox_Subsystem;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig.CornerID;
import frc.lib2202.util.PIDFController;
import frc.robot2024.subsystems.sensors.Sensors_Subsystem;
import static frc.lib2202.Constants.MperFT;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotSpec_DoofBot implements IRobotSpec {

    final SubsystemConfig ssConfig = new SubsystemConfig("2023-DoofBot", "TBD-123456-TBD")
            .add(Sensors_Subsystem.class)
            .add(Limelight.class)
            .add(HID_Xbox_Subsystem.class, "DC", () -> {
                return new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
            })
            .add(SwerveDrivetrain.class); // TODO fix doof specs

    // set this true at least once after robot hw stabilizes
    boolean burnFlash = false;
    boolean swerve = true;

    // Robot Speed Limits
    static double maxSpeed = 15.0 * MperFT; // [m/s]
    static double maxRotationRate = 2.0 * Math.PI; // [rad/s]
    static RobotLimits limits = new RobotLimits(maxSpeed, maxRotationRate);

    // Chassis
    static double kWheelCorrectionFactor = .995;
    static double kSteeringGR = 12.8;
    static double kDriveGR = 6.12;
    static double kWheelDiameter = MperFT * 4.0 / 12.0; // [m]

    public static final ChassisConfig doofBotChassisConfig = new ChassisConfig(
            MperFT * (23.5 / 12.0) / 2.0, // based on CAD in reference_links
            MperFT * (19.5 / 12.0) / 2.0, // based on CAD in reference_links
            kWheelCorrectionFactor, // scale [] <= 1.0
            MperFT * (4.0 / 12.0), // wheel diameter[m] Comp bot is 4" wheels
            kSteeringGR, // confirmed with vince
            kDriveGR,
            new PIDFController(0.085, 0.00055, 0.0, 0.21292),
            new PIDFController(0.01, 0.0, 0.0, 0.0) // angle
    );

    public RobotSpec_DoofBot() {
        ssConfig.setRobotSpec(this);
    }

    @Override
    public RobotLimits getRobotLimits() {
        return limits;
    }

    @Override
    public IHeadingProvider getHeadingProvider() {
        return RobotContainer.getSubsystem(Sensors_Subsystem.class);
    }

    @Override
    public ChassisConfig getChassisConfig() {
        return doofBotChassisConfig;
    }

    @Override
    public ModuleConfig[] getModuleConfigs() {

        // For 2023 CompetitionBot - Doof
        // public static final WheelOffsets doofBotOffsets = new WheelOffsets(129.03,
        // -83.94, -57.83, 139.38); //FL BL FR BR
        // from original constants
        // CANConfig doofBotCANConfig = new CANConfig(swerveBotCAN_FL, swerveBotCAN_FR,
        // swerveBotCAN_BL, swerveBotCAN_BR);
        // ModuleConfig comp2024CAN_FL = new ModuleConfig(29, 24, 25);
        // ModuleConfig comp2024CAN_FR = new ModuleConfig(30, 26, 27);
        // ModuleConfig comp2024CAN_BL = new ModuleConfig(28, 22, 23);
        // ModuleConfig comp2024CAN_BR = new ModuleConfig(31, 20, 21);
        // ChassisInversionSpecs doofBotChassisInversionSpecs = new
        // ChassisInversionSpecs(
        // new ModuleInversionSpecs(true, false, false), // FR
        // new ModuleInversionSpecs(false, false, false), // FL
        // new ModuleInversionSpecs(true, false, false), // BR
        // new ModuleInversionSpecs(false, false, false)); // BL

        ModuleConfig[] modules = new ModuleConfig[4];
        modules[CornerID.FrontLeft.getIdx()] = new ModuleConfig(CornerID.FrontLeft,
                29, 24, 25,
                129.03)
                .setInversions(false, false, false);

        modules[CornerID.FrontRight.getIdx()] = new ModuleConfig(CornerID.FrontRight,
                30, 26, 27,
                -57.83)
                .setInversions(true, true, false);

        modules[CornerID.BackLeft.getIdx()] = new ModuleConfig(CornerID.BackLeft,
                28, 22, 23,
                -83.94)
                .setInversions(false, false, false);

        modules[CornerID.BackRight.getIdx()] = new ModuleConfig(CornerID.BackRight,
                31, 20, 21,
                139.38)
                .setInversions(true, false, false);

        return modules;
    }

    @Override
    public void setBindings() {
        HID_Xbox_Subsystem dc = RobotContainer.getSubsystem("DC");
        // pick one of the next two lines
        BindingsCompetition.ConfigureCompetition(dc);
        // BindingsOther.ConfigureOther(dc);

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
       SwerveDrivetrain drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
        if (drivetrain != null) {
            drivetrain.setDefaultCommand(new FieldCentricDrive());
          }
    }


}
