package frc.robot2026;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static frc.lib2202.Constants.DEGperRAD;
import static frc.lib2202.Constants.MperFT;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
//import edu.wpi.first.math.geometry.Transform3d;
//import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
import frc.lib2202.command.swerve.FieldCentricDrive;
import frc.lib2202.subsystem.Odometry;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.Sensors;
import frc.lib2202.subsystem.UX.TrimTables;   
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.AutoPPConfigure;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig.CornerID;
import frc.lib2202.util.PIDFController;
import frc.robot2026.Constants.CAN;
import frc.robot2026.Constants.DigitalIO;
import frc.robot2026.command.pose.setGyroOffsetWithVision;
import frc.robot2026.subsystems.Climber;
import frc.robot2026.subsystems.Hopper;
import frc.robot2026.subsystems.Intake;
import frc.robot2026.subsystems.LimelightV2;
import frc.robot2026.subsystems.Photonvision;
//import frc.robot2026.subsystems.RangeSensor;
import frc.robot2026.subsystems.VisionPoseEstimator;
import frc.robot2026.subsystems.Shooter.Fuelgauge;
import frc.robot2026.subsystems.Shooter.Indexer;
import frc.robot2026.subsystems.Shooter.Shooter;
import frc.robot2026.subsystems.Shooter.Targeter;
import frc.robot2026.util.BetterAutoChooser;

public class RobotSpec_AlphaBot implements IRobotSpec {
  // Subsystem objects for use at other cut points
  Targeter targeter;   // auto/tele init
  static Photonvision pv = null;

  // copy source under deploy for this robot.
  final static String DEPLOY_DIR = "2026";

  // 2026 Robot rev Alpha
  // io sheet
  // https://docs.google.com/spreadsheets/d/1eZ89R4oWHoCDpM9nOMC420o4i6Zx-Fgi8y4tpiL58a4/edit?gid=2120414614#gid=2120414614
  // This should be the chassis bot.
  // $env:serialnum = "025AE07D"
  final SubsystemConfig ssconfig = new SubsystemConfig("AlphaBot2026", "025AE07D")
      // deferred construction via Supplier<Object> lambda
      .add(PowerDistribution.class, "PDP", () -> {
        var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
        pdp.clearStickyFaults();
        return pdp;
      })
      .add(HID_Subsystem.class, "DC", () -> {
        var hid =  new HID_Subsystem(0.3, 0.6, 0.05);
        hid.new HIDMonitorCmd();
        return hid;
      })
      // Sensors, limelight and drivetrain all use interfaces, so make sure their
      // alias names
      // match what is given here.
      .add(Sensors.class, "sensors", () -> {
        return new Sensors(CAN.PIGEON_IMU_CAN);
      })
      .add(TrimTables.class)
      .add(LimelightV2.class, "limelight", () -> {
        // Limelight position in robot coords - this has LL in the front of bot
        Pose3d LimelightPosition = new Pose3d(0.32, 0.00, 0.515,   //new position 3/20/26
            new Rotation3d(0.0, 15.0 / DEGperRAD, 0.0));
        return new LimelightV2("limelight", LimelightPosition);
      })
      .add(SwerveDrivetrain.class, "drivetrain", () -> {
        return new SwerveDrivetrain(SparkFlex.class);
      })
      //.add(RangeSensor.class) //works but not used
      .add(OdometryInterface.class, "odometry", () -> {
        var obj = new Odometry();
        obj.new OdometryWatcher();
        return obj;
      })
      // VisonPoseEstimator needs LL and Odometry, adds simplename and alias to lookup
      .add(Indexer.class, "indexer_left", () -> {
        return new Indexer(CAN.LIndexerID, true, DigitalIO.IndexerGateLeft);
      })
      .add(Indexer.class, "indexer_right", () -> {
        return new Indexer(CAN.RIndexerID, false, DigitalIO.IndexerGateRight);
      })
      .addAlias(VisionPoseEstimator.class, "vision_odo")
      .add(Shooter.class, "shooter_left", () -> {
        return new Shooter("left", Constants.CAN.ShooterIDLeft, false);
      })
      .add(Shooter.class, "shooter_right", () -> {
        return new Shooter("right", Constants.CAN.ShooterIDRight, true);
      })
      .add(Fuelgauge.class)
      .add(Intake.class, "intake", () -> {
        return new Intake();
      })
      .add(Climber.class, "climber", () -> {
         return new Climber(true);
       })
       /****************************
        * Removing PV until more testing. 

      .add(Photonvision.class, "photonvision", () -> {
        // create config object with our cameras and their positions
        // Photonvision uses WPI coordinates: https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
        // X to front, Y to left, Z up
        // Rotation is roll, pitch, yaw 
        // yaw is positive counterclockwise looking down on the robot
        Photonvision.Config pvConfig = new Photonvision.Config(
            // CAD: +x towards front, +y robot right, +z towards top
            // left camera (-11.477,-11.477, 17.198) (inches)
            // right camera (3.682, 7.839, 15.720)

            new String[] { "Left_Camera", "Right_Camera" },
            new Transform3d[] {
                new Transform3d(new Translation3d(-11.477 * 0.0254, 11.477 * 0.0254, 17.198 * 0.0254),
                    new Rotation3d( 0.0, 0.0, 90 / DEGperRAD )),
                new Transform3d(new Translation3d(3.682 * 0.0254, -7.839 * 0.0254, 15.720 * 0.0254),
                   new Rotation3d( 0.0, 0.00, -90 / DEGperRAD )),
            });
        // now setup our PV subsystem
        pv = new Photonvision(pvConfig);
        return pv;
      })
        *************************/
      .add(Hopper.class)
      .add(Targeter.class)
      ;

  // Robot Speed Limits
  RobotLimits robotLimits = new RobotLimits(FeetPerSecond.of(15.0), DegreesPerSecond.of(315.0));

  // Chassis
  double kWheelCorrectionFactor = 1.00;  // all wheels
  double kSteeringGR = 12.8;
  double kDriveGR = 5.36;
  double kWheelDiameter = MperFT * (3.93 / 12.0); // [m] worn wheels, 3.93" measured 3/24/26

  final ChassisConfig chassisConfig = new ChassisConfig(
      0.66 / 2.0, // x, as measured, 2/21/2026
      0.715 / 2.0, // y, as measured, 2/21/2026
      kWheelCorrectionFactor, // scale [] <= 1.0   applies to all wheels
      kWheelDiameter,
      kSteeringGR,
      kDriveGR,
      // DPL, AH, JW - 12.0 factor for rev 2026 firmware using Volts and not PCT-Pwr
      new PIDFController(0.085, 0.00055, 0.0, 12.0 * 0.21292), // drive
      new PIDFController(0.03, 0.0, 0.0, 0.0) // angle
  );

  public RobotSpec_AlphaBot() {
    // finish BetaBot's drivePIDF
    chassisConfig.drivePIDF.setIZone(0.2);
    // add the specs to the ssconfig
    ssconfig.setRobotSpec(this);
  }

  // Required method that use the specs above

  @Override
  public RobotLimits getRobotLimits() {
    return robotLimits;
  }

  @Override
  public IHeadingProvider getHeadingProvider() {
    return RobotContainer.getSubsystem("sensors");
  }

  @Override
  public ChassisConfig getChassisConfig() {
    return chassisConfig;
  }

  @Override
  public ModuleConfig[] getModuleConfigs() {
    ModuleConfig[] modules = new ModuleConfig[4];
    modules[CornerID.FrontLeft.getIdx()] = new ModuleConfig(CornerID.FrontLeft,
        CAN.FL_CANCoder, CAN.FL_Drive, CAN.FL_Angle, -21.44)//-19.42) // old -21.089 8/31/26 ds //older -20.474
        .setInversions(false, true, true);
    modules[CornerID.FrontRight.getIdx()] = new ModuleConfig(CornerID.FrontRight,
        CAN.FR_CANCoder, CAN.FR_Drive, CAN.FR_Angle, -104.86) //-103.54)  // old 8/31/26 -104.155) ds//older -105.737
        .setInversions(true, true, true);
        //.withWheelFactor(3.98/3.93);   shouldn't need this anymore - changed wheels out// replaced wheel / other wheels dia, applies only to FR

    modules[CornerID.BackLeft.getIdx()] = new ModuleConfig(CornerID.BackLeft,
        CAN.BL_CANCoder, CAN.BL_Drive, CAN.BL_Angle, 163.39)//-178.500) // old 8/31/26 -168.842) ds//older -163.305
        .setInversions(false, true, true);

    modules[CornerID.BackRight.getIdx()] = new ModuleConfig(CornerID.BackRight,
        CAN.BR_CANCoder, CAN.BR_Drive, CAN.BR_Angle, 103.52) //102.64) // old 8/31/26 102.023) ds//101.936 older
        .setInversions(true, true, true);

    return modules;
  }

  @Override
  public void setBindings() {
    String odometryName = "vision_odo"; // or novision "odometry"
    OdometryInterface odo = RobotContainer.getSubsystemOrNull(odometryName);
    VisionPoseEstimator vpe = RobotContainer.getSubsystemOrNull("vision_odo");
    DriveTrainInterface sdt = RobotContainer.getSubsystemOrNull("drivetrain");
    HID_Subsystem dc = RobotContainer.getSubsystem("DC");

    // quiet the phoenix 6 logger noise
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();

    @SuppressWarnings("unused")
    CommandXboxController operator = (CommandXboxController)dc.Operator();

    //save for use in tele or auto init
    targeter = RobotContainer.getSubsystem(Targeter.class);
    
    // Initialize PathPlanner, if we have needed Subsystems
    if (odo != null && sdt != null) {
      AutoPPConfigure.configureAutoBuilder(sdt, odo,
            new PIDConstants(3.0, 0.0, 0.0),  // Translation PID constants,
            new PIDConstants(5.0, 0.0, 0.0)); // Rotation PID constants | P was 7.0);
      var cmd = PathfindingCommand.warmupCommand();
      CommandScheduler.getInstance().schedule(cmd);
    }

    // Competition bindings
    BindingsCompetition.ConfigureCompetition(dc, true);

    // Place your test binding in ./testBinding/<yourFile>.java and call it here
    // comment out any conflicting bindings. Try not to push with your bindings
    // active. Just comment them out.   
    // DpltestBinding.calbrate((CommandXboxController)dc.Operator());
    // BGTestBindings.calbrate(op);
    // GRLTestBindings.calbrate((CommandXboxController)dc.Operator()); //uses opr a & b
     //JRtestBinding.calbrate((CommandXboxController)dc.Driver());
   
    // Anything else that needs to run after binding/commands are created 
    if (vpe != null) {
      vpe.configureGyroCallback();
    }

    // show what cmds are running
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  // auto paths in the 2026 folder, set up top.
  @Override
  public String getDeployDirectory() {
        return DEPLOY_DIR;
  }

  SendableChooser<Command> autoChooser;

  @Override
  public void setupRegisteredCommands() {
    RegisteredCommands.RegisterCommands();

    // Builds autochooser list from PathPlanner's autos.
    // Requires AutoBuilder to be configured thus SDT and some form of odometry.
    // Skip auto if not configured.
    if (AutoBuilder.isConfigured()) {
      autoChooser = BetterAutoChooser.buildAutoChooser();
     
      SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // if needed other auto commands can be added here
    // autoChooser.addOption(Name:, cmd);

  }

  @Override
  public SendableChooser<Command> getChooser() {
    return autoChooser;
  }

  @Override
  public void setDefaultCommands() {
    DriveTrainInterface drivetrain = RobotContainer.getSubsystemOrNull("drivetrain");
    if (drivetrain != null) {
      drivetrain.setDefaultCommand(new FieldCentricDrive());
    }
  }

  //Handle things on tele or auto init

  @Override
  public void teleopInit() {   
    IRobotSpec.super.teleopInit();
    targeter.setTarget(); // set blue/red alliance

    //TODO - TEST CODE, REMOVE
    //CommandScheduler.getInstance().schedule(new TargetCentricDrive(targeter));
  }

  @Override
  public void autonomousInit(){
    IRobotSpec.super.autonomousInit();
    targeter.setTarget(); // set blue/red alliance    
  }


  @Override
  public void disabledInit(){
    CommandScheduler.getInstance().schedule(new setGyroOffsetWithVision());
  }

}