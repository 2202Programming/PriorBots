package frc.robot2026;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static frc.lib2202.Constants.DEGperRAD;
import static frc.lib2202.Constants.MperFT;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
import frc.lib2202.command.ScaleDriver;
import frc.lib2202.command.pathing.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.FieldCentricDrive;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.command.swerve.RotateTo;
import frc.lib2202.command.swerve.TargetCentricDrive;
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
import frc.robot2026.command.pose.setGyroOffsetWithVision;
import frc.robot2026.subsystems.LimelightV2;
import frc.robot2026.subsystems.Photonvision;
import frc.robot2026.subsystems.VisionPoseEstimator;
import frc.robot2026.subsystems.Shooter.Targeter;

public class RobotSpec_ChassisBot implements IRobotSpec {
  // Subsystems and other hardware on 2025 Robot rev Alpha
  // This should be the chassis bot.
  // $env:serialnum = "03282B65"

   static Photonvision pv;

  final SubsystemConfig ssconfig = new SubsystemConfig("ChassisBot", "03282B65")
      // deferred construction via Supplier<Object> lambda
      .add(PowerDistribution.class, "PDP", () -> {
        var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
        pdp.clearStickyFaults();
        return pdp;
      })
      .add(HID_Subsystem.class, "DC", () -> {
        return new HID_Subsystem(0.3, 0.9, 0.05);
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
        // WARNING: LL has +Y to the right, normal wpi robot coords are +Y to left
        Pose3d LimelightPosition = new Pose3d(0.24, 0.38, .22865, 
            new Rotation3d(0.0, 11.0 / DEGperRAD, -90.0 / DEGperRAD));
        return new LimelightV2("limelight", LimelightPosition);

      })
      .add(SwerveDrivetrain.class, "drivetrain", () -> {
        return new SwerveDrivetrain(SparkFlex.class);
      })
      /*
      .add(Photonvision.class, "photonvision", () -> {
        // create config object with our cameras and their positions
        Photonvision.Config pvConfig = new Photonvision.Config(
            // Fairly confident about x/y/z/yaw, not sure about pitch/roll for back cameras
            // Measurements:
            // All cameras 9.4 cm off ground
            // Front Camera:
            // 23 cm in front of robot center
            // 19 cm to the right (negative) of robot center
            // Angled 2deg up from horizon
            //
            // Back Right
            // 14 cm in front of robot center
            // 25 cm to the right of robot center
            // Angled 11deg up from horizon
            //
            // Back Left
            // 13 cm in front of robot center
            // 13 cm right of robot center
            // Angled 11deg up from horizon 
            new String[] { "Back_Left", "back_right", "Front" },
            new Transform3d[] {
              //dpl - these are what I got on 2/19/26
              //    - the post kind of moves so pitch could be different if bumped.
                new Transform3d(new Translation3d(0.13, -0.17, 0.29),
                    new Rotation3d( 0.0, 7.0 /DEGperRAD, 120.0 / DEGperRAD )),
                new Transform3d(new Translation3d(.13, -.21, .29),
                   new Rotation3d( 0.0, 7.0 /DEGperRAD, -120.0 / DEGperRAD )),
                new Transform3d(new Translation3d(.16, -.19, .29),
                    new Rotation3d(0., 7.0/DEGperRAD, 0.0))
            });
        // now setup our PV subsystem
        pv = new Photonvision(pvConfig);
        return pv;
      })
        */
      .add(OdometryInterface.class, "odometry", () -> {
        var obj = new Odometry();
        obj.new OdometryWatcher();
        return obj;
      })
      // VisonPoseEstimator needs LL and Odometry, adds simplename and alias to lookup
      .addAlias(VisionPoseEstimator.class, "vision_odo")
      .add(Targeter.class)
      ;

  // Robot Speed Limits
  RobotLimits robotLimits = new RobotLimits(FeetPerSecond.of(15.0), DegreesPerSecond.of(270.0));

  // Chassis
  double kWheelCorrectionFactor = .9875;
  double kSteeringGR = 21.428;
  double kDriveGR = 6.12;
  double kWheelDiameter = MperFT * 4.0 / 12.0; // [m]

  final ChassisConfig chassisConfig = new ChassisConfig(
      0.58 / 2.0,  //coord front-left x
      0.58 / 2.0,  //coord front-left y
      kWheelCorrectionFactor, // scale [] <= 1.0
      kWheelDiameter,
      kSteeringGR,
      kDriveGR,
      //fix kf for rev voltage ctrl and not pct-pwr
      new PIDFController(0.085, 0.00055, 0.0, 12.0*0.21292), // drive
      new PIDFController(0.01, 0.0, 0.0, 0.0) // angle
  );

  public RobotSpec_ChassisBot() {
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
    // TODO - correct offsets
    ModuleConfig[] modules = new ModuleConfig[4];
    modules[CornerID.FrontLeft.getIdx()] = new ModuleConfig(CornerID.FrontLeft,
        CAN.FL_CANCoder, CAN.FL_Drive, CAN.FL_Angle, -21.26)//41.76
        .setInversions(false, true, false);

    modules[CornerID.FrontRight.getIdx()] = new ModuleConfig(CornerID.FrontRight,
        CAN.FR_CANCoder, CAN.FR_Drive, CAN.FR_Angle, -108.46)//new Cancoder 2/21/26 //-71.62
        .setInversions(true, true, false);

    modules[CornerID.BackLeft.getIdx()] = new ModuleConfig(CornerID.BackLeft,
        CAN.BL_CANCoder, CAN.BL_Drive, CAN.BL_Angle, -131.40)//50.45
        .setInversions(false, true, false);

    modules[CornerID.BackRight.getIdx()] = new ModuleConfig(CornerID.BackRight,
        CAN.BR_CANCoder, CAN.BR_Drive, CAN.BR_Angle, 105.37)//-66.27
        .setInversions(true, true, false);

    return modules;
  }

  @Override
  public void setBindings() {
    // String odometryName = VisionPoseEstimator.class.getSimpleName(); // or
    // novision "odometry"
    // TODO switch to vision based when we have a LL
    OdometryInterface odo = RobotContainer.getSubsystemOrNull("odometry");
    VisionPoseEstimator vpe = RobotContainer.getSubsystemOrNull("vision_odo");
    DriveTrainInterface sdt = RobotContainer.getSubsystemOrNull("drivetrain");
    HID_Subsystem dc = RobotContainer.getSubsystem("DC");
    //Climber cl = RobotContainer.getSubsystem("climber");
    vpe.getWatcher();

    // Initialize PathPlanner, if we have needed Subsystems
    if (odo != null && sdt != null) {
      AutoPPConfigure.configureAutoBuilder(sdt, vpe);
      var cmd = PathfindingCommand.warmupCommand();
      CommandScheduler.getInstance().schedule(cmd);
    }

    // Competition bindings
    localBindings(dc);
    
    // maybe beter way, but this registers vpe with the aliance-aware reset cmd.
    if (vpe != null) vpe.configureGyroCallback();

    // show what cmds are running
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  SendableChooser<Command> autoChooser;

  @Override
  public void setupRegisteredCommands() {
    // likely not enough on this bot for registerd cmds
    //RegisteredCommands.RegisterCommands();

    // enable chooser - builds autochooser list, requires AutoBuilder to be
    // configured
    // thus SDT and some form of odometry. Skip auto if not configured.
    if (AutoBuilder.isConfigured()) {
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
    }
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

  @Override
  public void disabledInit(){
    CommandScheduler.getInstance().schedule(new setGyroOffsetWithVision());
  }


private void localBindings(HID_Subsystem _dc)
{
   DriveTrainInterface drivetrain = RobotContainer.getSubsystem("drivetrain");  
   Targeter targeter = RobotContainer.getSubsystem(Targeter.class);
   var generic_driver = _dc.Driver();

  if (generic_driver instanceof CommandXboxController) {
            // XBox
            CommandXboxController driver = (CommandXboxController) generic_driver;
            driver.rightBumper().whileTrue(new RobotCentricDrive(drivetrain, _dc));
            driver.back().whileTrue(new TargetCentricDrive(targeter.getRedHub(), targeter.getBlueHub()) 
                                    .setP(4.0));
            // testing on rotate to target
            driver.start().onTrue(new RotateTo(targeter.getRedHub(),
                                               targeter.getBlueHub(),1.0)
                                               .setP(4.0));

            driver.y().onTrue(new AllianceAwareGyroReset());

            // Driver will wants precision robot-centric throttle drive on left bumper
            driver.leftBumper().whileTrue(new ParallelCommandGroup(
                    new ScaleDriver(0.3),
                    new RobotCentricDrive(drivetrain, _dc)));
   } else {
    DriverStation.reportError("Comp Bindings: No driver bindings set, check controllers.", false);
  }

}


}