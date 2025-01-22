package frc.robot2019;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static frc.lib2202.Constants.MperFT;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.robot2019.commands.CommandManager;
import frc.robot2019.commands.CommandManager.Modes;
/*
import frc.lib2202.util.PIDFController;
import frc.robot2019.Constants;  // some added constants for port
import frc.robot2019.RobotMap;  // most of the constants from that year
import frc.robot2019.OI;        // bindings  old way of doing things
*/
import frc.robot2019.subsystems.ArmSubsystem;
import frc.robot2019.subsystems.CameraSubsystem;
import frc.robot2019.subsystems.CargoTrapSubsystem;
import frc.robot2019.subsystems.ClimberSubsystem;
import frc.robot2019.subsystems.DriveTrainSubsystem;
import frc.robot2019.subsystems.GearShifterSubsystem;
import frc.robot2019.subsystems.IntakeSubsystem;
import frc.robot2019.subsystems.SensorSubsystem;  


public class RobotSpec_2019 implements IRobotSpec {

    boolean teleOpRunOnce = true;

    // Create list of subsystems this robot uses, remember their periodic() are called in order of this list
    final SubsystemConfig config = new SubsystemConfig(
            "Robot_2019", "fixme")
            // deferred construction via Supplier<Object> lambda
            .add(PowerDistribution.class, "PDP", () -> {
                var pdp = new PowerDistribution(RobotMap.CAN_PDP_ID, ModuleType.kCTRE);  //todo check if rev or ctre
                pdp.clearStickyFaults();
                return pdp;
            })
            .add(DriveTrainSubsystem.class)
            .add(GearShifterSubsystem.class)
            .add(ArmSubsystem.class)
            .add(CameraSubsystem.class)
            .add(IntakeSubsystem.class)
            .add(CargoTrapSubsystem.class)
            .add(IntakeSubsystem.class)
            .add(SensorSubsystem.class)  //creates SerialPortSS and LimeLightSS inside
            //.add(SerialPortSubsystem.class)
            //.add(LimeLightSubsystem.class)
            .add(ClimberSubsystem.class)    // no longer exists on robot
            // non-subystem objects
            .add(OI.class, "OI", () -> {return new OI(); })  //lots of bindings here, uses subsystems
            // setup cmd mgr - code copied from Robot.old_reference
            .add(CommandManager.class, "CommandManager", () -> {
                var m_cmdMgr = new CommandManager();
                //not sure if both are really needed
                m_cmdMgr.setMode(Modes.Construction);   // schedules the mode's function
                m_cmdMgr.setMode(Modes.SettingZeros);   // schedules the mode's function    
                return m_cmdMgr;
            } )
            ;  //end of subsystems

    boolean swerve = false;

    // Robot Speed Limits -  TODO, make this this bot's drive command use these limits
    // swerve library uses these by default, but 2019 likely hardwired in various places
    RobotLimits robotLimits = new RobotLimits(FeetPerSecond.of(15.0), DegreesPerSecond.of(180.0));
    
    // Chassis
    double kWheelCorrectionFactor = 1.0;
    double kSteeringGR = 21.428;
    double kDriveGR = 6.12;
    double kWheelDiameter = MperFT * 4.0 / 12.0; // [m]

    final ChassisConfig chassisConfig = new ChassisConfig(
            //TODO - fix numbers they could still be used for corner definitions on a non-swerve bot
            MperFT * (24.875 / 12.0) / 2.0, // x
            MperFT * (20.5 / 12.0) / 2.0, // y
            kWheelCorrectionFactor, // scale [] <= 1.0
            kWheelDiameter,
            kSteeringGR,
            kDriveGR,
            // don't new these pids, not a swerve bot.
            null, //new PIDFController(0.085, 0.00055, 0.0, 0.21292), // drive
            null //new PIDFController(0.01, 0.0, 0.0, 0.0) // angle
    );

    public RobotSpec_2019() {        
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
        return RobotContainer.getSubsystem(SensorSubsystem.class);
    }

    @Override
    public ChassisConfig getChassisConfig() {
        return chassisConfig;
    }

    @Override
    public ModuleConfig[] getModuleConfigs() {
        // not a swerve bot, null is ok
        return null;
    }

    @Override
    public void setBindings() {
        //TODO FIX THIS
    }


    @Override
    public SendableChooser<Command> getRegisteredCommands() {      
        // used sideboard buttons this year and not Dashboard UX chooser  
        return null;
    }

    @Override
    public void setDefaultCommands() {
        // subsystems might set their own default commands from old initDefaultCommand()
      
    }
    @Override
    public void teleopInit(){         
        if (teleOpRunOnce) {
            // add cmd and schedule here as needed
            teleOpRunOnce = false;
        }
        //add any other cmds/schedule as needed 
    }
}
