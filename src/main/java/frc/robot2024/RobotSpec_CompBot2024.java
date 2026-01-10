package frc.robot2024;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static frc.lib2202.Constants.MperFT;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
import frc.lib2202.command.swerve.FieldCentricDrive;
import frc.lib2202.subsystem.BlinkyLights;
import frc.lib2202.subsystem.Limelight;
import frc.lib2202.subsystem.Odometry;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.Sensors;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.AutoPPConfigure;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig.CornerID;
import frc.lib2202.util.PIDFController;
import frc.robot2024.Constants.CAN;
import frc.robot2024.commands.Shooter.CalibrateWithLS;
import frc.robot2024.commands.Shooter.DistanceInterpretor;
import frc.robot2024.subsystems.AmpMechanism;
import frc.robot2024.subsystems.Climber;
import frc.robot2024.subsystems.Intake;
import frc.robot2024.subsystems.ShooterServo;
import frc.robot2024.subsystems.Transfer;


public class RobotSpec_CompBot2024 implements IRobotSpec {
    final static String DEPLOY_DIR = "2024";

    boolean teleOpRunOnce = true;

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
            .add(HID_Subsystem.class, "DC", () -> {
                return new HID_Subsystem(0.3, 0.9, 0.05);
            })
            .addAlias(Sensors.class,"sensors")
            .addAlias(Limelight.class, "limelight")
            .addAlias(SwerveDrivetrain.class, "drivetrain") // must be after LL and Sensors
            .add(OdometryInterface.class, "odometry", () -> {
                var obj = new Odometry();
                obj.new OdometryWatcher();
                return obj;
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

    boolean swerve = true;

    // Robot Speed Limits
    double maxSpeedFPS = 15.0; // [ft/s] 
    double maxRotationRateDPS = 360.0; // [deg/s]
    RobotLimits robotLimits = new RobotLimits(FeetPerSecond.of(15.0), DegreesPerSecond.of(180.0));
    
    // Chassis
    double kWheelCorrectionFactor = .987;
    double kSteeringGR = 21.428;
    double kDriveGR = 6.12;
    double kWheelDiameter = MperFT * 4.0 / 12.0; // [m]

    final ChassisConfig comp2024BotBetaChassisConfig = new ChassisConfig(
            //MperFT * (24.875 / 12.0) / 2.0, // x
            //MperFT * (20.5 / 12.0) / 2.0, // y
            (.655 / 2.0), // x [m]    updated from 2025 measurments
            (.755 / 2.0), // y [m]
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
        return RobotContainer.getSubsystem(Sensors.class);
    }

    @Override
    public ChassisConfig getChassisConfig() {
        return comp2024BotBetaChassisConfig;
    }

    @Override
    public ModuleConfig[] getModuleConfigs() {
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
                -150.380) //28.125)
                //dpl/ds -11/17/25 invert dr-mtr, works now on hw.
                .setInversions(false, true, false);

        modules[CornerID.BackRight.getIdx()] = new ModuleConfig(CornerID.BackRight,
                31, 20, 21,
                -115.752)
                .setInversions(false, true, false);

        return modules;
    }

    @Override
    public void setBindings() {
        HID_Subsystem dc = RobotContainer.getSubsystem("DC");
        OdometryInterface odo = RobotContainer.getSubsystemOrNull("odometry");
        DriveTrainInterface sdt = RobotContainer.getSubsystemOrNull("drivetrain");
        
        // Initialize PathPlanner, if we have needed Subsystems
        if (odo != null && sdt != null) {
            AutoPPConfigure.configureAutoBuilder(sdt, odo);
            var cmd = PathfindingCommand.warmupCommand();
            CommandScheduler.getInstance().schedule(cmd);
        }
        
        // pick one of the next two lines
        BindingsCompetition.ConfigureCompetition(dc);
        // BindingsOther.ConfigureOther(dc);
    }
    
    // allow our bot's 2024 deploy folder to get copied to correct spot
    // this a multi-robot, multi-deploy folder thing.  Don't use on competition bots.
    @Override
    public String getDeployDirectory() {
        return DEPLOY_DIR;
    }

    // setup reg commands and relate autoChooser
    SendableChooser<Command> autoChooser;
    @Override
    public void setupRegisteredCommands() {
        autoChooser = RegisteredCommands.RegisterCommands();
    }

    @Override
    public SendableChooser<Command> getChooser() { 
        return autoChooser;
    }


    @Override
    public void setDefaultCommands() {
       SwerveDrivetrain drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
        if (drivetrain != null) {
            drivetrain.setDefaultCommand(new FieldCentricDrive());
            //RobotCentricDrive());
          }
    }
    @Override
    public void teleopInit(){
         // Temp command for compbot2024 to calibrate shooter's servo
        if (teleOpRunOnce) {
            // ensure shooter is calibrated on power up - note for a competition this
            // should not be needed and the bot should be calibrated in the pit
            var cmd = new CalibrateWithLS();
            CommandScheduler.getInstance().schedule(cmd);
            teleOpRunOnce = false;
        }
    }
}
