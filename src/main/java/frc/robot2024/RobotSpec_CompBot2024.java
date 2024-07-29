package frc.robot2024;

import static frc.lib2202.Constants.MperFT;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
import frc.lib2202.command.swerve.FieldCentricDrive;
import frc.lib2202.subsystem.BlinkyLights;
import frc.lib2202.subsystem.Limelight;
import frc.lib2202.subsystem.hid.HID_Xbox_Subsystem;
import frc.lib2202.subsystem.swerve.DTMonitorCmd;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig.CornerID;
import frc.lib2202.util.PIDFController;
import frc.robot2024.Constants.CAN;
import frc.robot2024.commands.Shooter.DistanceInterpretor;
import frc.robot2024.subsystems.AmpMechanism;
import frc.robot2024.subsystems.Climber;
import frc.robot2024.subsystems.Intake;
import frc.robot2024.subsystems.ShooterServo;
import frc.robot2024.subsystems.Transfer;
import frc.robot2024.subsystems.sensors.Sensors_Subsystem;

public class RobotSpec_CompBot2024 implements IRobotSpec {

    final SubsystemConfig config = new SubsystemConfig(
            "CompetitionBotBeta2024", "032D2062")
            // deferred construction via Supplier<Object> lambda
            .add(PowerDistribution.class, "PDP", () -> {
                var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
                pdp.clearStickyFaults();
                return pdp;
            })
            .add(BlinkyLights.class, "LIGHTS", () -> {
                return new BlinkyLights(CAN.CANDLE1, CAN.CANDLE2);
            })
            .add(HID_Xbox_Subsystem.class, "DC", () -> {
                return new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
            })
            .add(Sensors_Subsystem.class)
            .add(Limelight.class)
            .add(SwerveDrivetrain.class) // must be after LL and Sensors
            .add(Command.class, "DT_Monitor", () -> {
                return new DTMonitorCmd();
            })
            .add(Intake.class)
            .add(Command.class, "IntakeWatcher", () -> {
                return RobotContainer.getSubsystem(Intake.class).getWatcher();
            })
            .add(Transfer.class)
            .add(ShooterServo.class)
            .add(Climber.class)
            .add(AmpMechanism.class)
            .add(Command.class, "ClimberWatcher", () -> {
                return RobotContainer.getSubsystem(Climber.class).getWatcher();
            })
            .add(Command.class, "ShooterServoWatcher", () -> {
                // cast to get the correct type of shooter
                return (RobotContainer.getSubsystem(ShooterServo.class)).getWatcher();
            })
            .add(Command.class, "TransferWatcher", () -> {
                return RobotContainer.getSubsystem(Transfer.class).getWatcher();
            })
            .add(DistanceInterpretor.class, "TargetingTable", () -> {
                return DistanceInterpretor.getSingleton();
            });

    // set this true at least once after robot hw stabilizes
    boolean burnFlash = false;
    boolean swerve = true;

    // Robot Speed Limits
    double maxSpeed = 15.0 * MperFT; // [m/s]
    double maxRotationRate = 2.0 * Math.PI; // [rad/s]
    RobotLimits robotLimits = new RobotLimits(maxSpeed, maxRotationRate);

    // Chassis
    double kWheelCorrectionFactor = .987;
    double kSteeringGR = 21.428;
    double kDriveGR = 6.12;
    double kWheelDiameter = MperFT * 4.0 / 12.0; // [m]

    final ChassisConfig comp2024BotBetaChassisConfig = new ChassisConfig(
            MperFT * (24.875 / 12.0) / 2.0, // x
            MperFT * (20.5 / 12.0) / 2.0, // y
            kWheelCorrectionFactor, // scale [] <= 1.0
            kWheelDiameter,
            kSteeringGR,
            kDriveGR,
            new PIDFController(0.085, 0.00055, 0.0, 0.21292), // drive
            new PIDFController(0.01, 0.0, 0.0, 0.0) // angle
    );

    public RobotSpec_CompBot2024() {
        // finish BetaBot's drivePIDF
        comp2024BotBetaChassisConfig.drivePIDF.setIZone(0.2);

        // finally add this spec to the config
        config.setRobotSpec(this);
    }

    // Required method that use the specs above

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
        return comp2024BotBetaChassisConfig;
    }

    @Override
    public ModuleConfig[] getModuleConfigs() {
        // from original constants
        // WheelOffsets(-125.595, 28.125, -114.785, -115.752); //FL BL FR BR
        // final ModuleConfig comp2024CAN_FL = new ModuleConfig(29, 24, 25);
        // final ModuleConfig comp2024CAN_FR = new ModuleConfig(30, 26, 27);
        // final ModuleConfig comp2024CAN_BL = new ModuleConfig(28, 22, 23);
        // final ModuleConfig comp2024CAN_BR = new ModuleConfig(31, 20, 21);
        // final CANConfig comp2024BotCANConfig = new CANConfig(comp2024CAN_FL,
        // comp2024CAN_FR, comp2024CAN_BL, comp2024CAN_BR);

        // ChassisInversionSpecs comp2024BotBetaInversionSpecs = new
        // ChassisInversionSpecs(
        // new ModuleInversionSpecs(false, true, false), // FR
        // new ModuleInversionSpecs(true, true, false), // FL
        // new ModuleInversionSpecs(false, true, false), // BR
        // new ModuleInversionSpecs(true, true, false)); // BL

        ModuleConfig[] modules = new ModuleConfig[4];
        modules[CornerID.FrontLeft.getIdx()] = new ModuleConfig(CornerID.FrontLeft,
                29, 24, 25,
                -125.595)
                .setInversions(true, true, false);

        modules[CornerID.FrontRight.getIdx()] = new ModuleConfig(CornerID.FrontRight,
                30, 26, 27,
                -114.785)
                .setInversions(false, true, false);

        modules[CornerID.BackLeft.getIdx()] = new ModuleConfig(CornerID.BackLeft,
                28, 22, 23,
                28.125)
                .setInversions(true, true, false);

        modules[CornerID.BackRight.getIdx()] = new ModuleConfig(CornerID.BackRight,
                31, 20, 21,
                -115.752)
                .setInversions(false, true, false);

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
        return RegisteredCommands.RegisterCommands();
    }

    @Override
    public void setDefaultCommands() {
       SwerveDrivetrain drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
        if (drivetrain != null) {
            drivetrain.setDefaultCommand(new FieldCentricDrive());
          }
    }

}
