package frc.robot2025;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static frc.lib2202.Constants.MperFT;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.robot2025.Constants.CAN;
import frc.robot2025.subsystems.demo.CapstanDrive;
import frc.robot2025.subsystems.demo.CycloidalDrive;
import frc.robot2025.subsystems.demo.SimpleServo;

//copy or extend this code for your robot - remember to override:
// TBD
//
public class RobotSpec_BotOnBoard2 implements IRobotSpec {

    final static String DEPLOY_DIR = "BotOnBoard02";

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
            kSteeringGR, // []
            kDriveGR); // []

    // SubsystemConfig gets registered in static array to match serial number at
    // Construct calls.  To debug in Sim remember to set the env-var in vscode debug window:
    //      $env:serialnum = "0312db1a"
    //      $env:serialnum = "03061025"  # for cycloidalDrive

    static SimpleServo Servo0;
    static CycloidalDrive Cycloid0;
    static CapstanDrive Capstan0;

    SubsystemConfig subsystemConfig = new SubsystemConfig("bot-On-Board-2", "03061025")
            // deferred construction via Supplier<Object> lambda
            .add(PowerDistribution.class, "PDP", () -> {
                var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
                pdp.clearStickyFaults();
                return pdp;
            })
            .add(HID_Subsystem.class, "DC", () -> {
                return new HID_Subsystem(0.3, 0.9, 0.05);
            })   
            .add(SimpleServo.class, "Servo0", () -> {
                // save ref for binding cmds later
                Servo0 = new SimpleServo(0);
                Servo0.getWatcherCmd();
                return Servo0;
            })
            .add(CycloidalDrive.class, "CycloidalDrive", () -> {       
                Cycloid0 = new CycloidalDrive(55);
                Cycloid0.getWatcherCmd();
                return Cycloid0;
            })        
            .add(CapstanDrive.class, "CapstanDrive", () -> {
                Capstan0 = new CapstanDrive(18);
                Capstan0.getWatcherCmd();
                return Capstan0;
            })
    ;

    public RobotSpec_BotOnBoard2() {
        // finish BetaBot's drivePIDF
        chassisConfig.drivePIDF.setIZone(0.2);
        // add the specs to the ssconfig
        subsystemConfig.setRobotSpec(this);
    }
   
    @Override
    public void setBindings() {
        HID_Subsystem dc = RobotContainer.getSubsystem("DC");
        
        //@SuppressWarnings("unused")
        CommandXboxController driver = (CommandXboxController) dc.Driver();
        CommandXboxController operator = (CommandXboxController) dc.Operator();
       
        //Add your bindings here
        // bindings for simple servo demo
        driver.a().onTrue(Servo0.cmdPosition(0.0));
        driver.x().onTrue(Servo0.cmdPositionWaitForModel(0.5));
        driver.b().onTrue(Servo0.cmdPositionWaitForModel(1.0));

        //bindings for Cycloid demo - uses POV and rtTrigger, L/R Bumper
        Cycloid0.setDemoBindings(driver);   // uses driver controller
        //TylerCommands.myBindings(dc);       // uses operator controller

        Capstan0.setDemoBindings(operator);

        //show what commands are running
        SmartDashboard.putData(CommandScheduler.getInstance());
    }
    
    // uncomment for multi-bot repo, leave commented out for a competiton repo.
     @Override
    public String getDeployDirectory() {
        return DEPLOY_DIR;
    }

}
