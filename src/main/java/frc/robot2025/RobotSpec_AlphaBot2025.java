package frc.robot2025;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static frc.lib2202.Constants.DEGperRAD;
import static frc.lib2202.Constants.MperFT;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
import frc.lib2202.command.swerve.FieldCentricDrive;
import frc.lib2202.subsystem.Odometry;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.AutoPPConfigure;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig.CornerID;
import frc.lib2202.util.PIDFController;
import frc.robot2025.Constants.CAN;
import frc.robot2025.subsystems.Limelight;
import frc.robot2025.subsystems.Sensors_Subsystem;
import frc.robot2025.subsystems.VisionPoseEstimator;
import frc.robot2025.testBindings.DPLPathTest;
import frc.robot2025.utils.UXTrim;

public class RobotSpec_AlphaBot2025 implements IRobotSpec {

  // Subsystems and other hardware on 2025 Robot rev Alpha
  // $env:serialnum = "03282B65"
  final SubsystemConfig ssconfig = new SubsystemConfig("AlphaBot2025", "03282B65")
      // deferred construction via Supplier<Object> lambda
      .add(PowerDistribution.class, "PDP", () -> {
        var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
        pdp.clearStickyFaults();
        return pdp;
      })
      /*
      // .add(PneumaticsControl.class)
      .add(BlinkyLights.class, "LIGHTS", () -> {
        return new BlinkyLights(CAN.CANDLE1, CAN.CANDLE2, CAN.CANDLE3, CAN.CANDLE4);
      })
        */
      .add(HID_Subsystem.class, "DC", () -> {
        return new HID_Subsystem(0.3, 0.9, 0.05);
      })
      /*/
      .add(GroundIntake.class)
      .add(Elevator_Subsystem.class)
      .add(Command.class, "ElevatorWatcher", () -> {
       return RobotContainer.getSubsystem(Elevator_Subsystem.class).getWatcher();
      })
*/
      // Sensors, limelight and drivetrain all use interfaces, so make sure their alias names
      // match what is given here.
      .add(Sensors_Subsystem.class, "sensors")
      .add(Limelight.class, "limelight", ()-> {
        // Limelight position in robot coords - this has LL in the front of bot
        Pose3d LimelightPosition = new Pose3d(0.7112 / 2.0, -0.21, .23,
          new Rotation3d(0.0, 12.0/DEGperRAD, 0.0));
        return new Limelight("limelight", LimelightPosition );
      })
      .add(SwerveDrivetrain.class, "drivetrain", () ->{
          return new SwerveDrivetrain(SparkFlex.class);
      })
      .add(OdometryInterface.class, "odometry", () -> {
        var obj = new Odometry();
        obj.new OdometryWatcher();
        return obj;
      })
      // VisonPoseEstimator needs LL and Odometry, adds simplename and alias to lookup
      .addAlias(VisionPoseEstimator.class, "vision_odo")
      // below are optional watchers for shuffeleboard data - disable if need too.
      /*
      .add(WristFLA.class)
      .add(SignalLight.class, "signal")
      .add(EndEffector_Subsystem.class)
      .add(Command.class, "endEffectorWatcher", () -> {
        return RobotContainer.getSubsystem(EndEffector_Subsystem.class).getWatcher();
      })
      .add(PDPMonitorCmd.class, ()->{ return new PDPMonitorCmd(); })
      */
      ;

  // Robot Speed Limits
  RobotLimits robotLimits = new RobotLimits(FeetPerSecond.of(15.0), DegreesPerSecond.of(180.0));

  // Chassis
  double kWheelCorrectionFactor = 1.02;
  double kSteeringGR = 21.428;
  double kDriveGR = 6.12;
  double kWheelDiameter = MperFT * 4.0 / 12.0; // [m]


  final ChassisConfig chassisConfig = new ChassisConfig(
      //0.57785 / 2.0, 
      //0.57785 / 2.0,  
      //dpl - 28" x 28"
      0.7112 / 2.0,  // x,  
      0.7112 / 2.0,  // y, 
      kWheelCorrectionFactor, // scale [] <= 1.0
      kWheelDiameter,
      kSteeringGR,
      kDriveGR,
      new PIDFController(0.085, 0.00055, 0.0, 0.21292), // drive
      new PIDFController(0.01, 0.0, 0.0, 0.0) // angle
  );

   

  public RobotSpec_AlphaBot2025() {
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
        // the rotation below was done and that's why the CAN IDs don't match what's in constants
        //A rotation was done below and CAN IDs don't match other names - THIS IS OKAY, DO NOT CHANGE THEM -- DPL + BG
        //FL -> BL
        //FR -> FL
        //BL -> BR
        //BR -> FR

        modules[CornerID.FrontLeft.getIdx()] = new ModuleConfig(CornerID.FrontLeft,
        CAN.FR_CANCoder, CAN.FR_Drive, CAN.FR_Angle, -155.390531)
        .setInversions(false, true, false);

        modules[CornerID.FrontRight.getIdx()] = new ModuleConfig(CornerID.FrontRight,
        CAN.BR_CANCoder, CAN.BR_Drive, CAN.BR_Angle, 24.873296)
        .setInversions(true, true, false);

        modules[CornerID.BackLeft.getIdx()] = new ModuleConfig(CornerID.BackLeft,
        CAN.FL_CANCoder, CAN.FL_Drive, CAN.FL_Angle, 133.153921)
        .setInversions(false, true, false);

        modules[CornerID.BackRight.getIdx()] = new ModuleConfig(CornerID.BackRight,
        CAN.BL_CANCoder, CAN.BL_Drive, CAN.BL_Angle,  -40.605625)
        .setInversions(true, true, false);

    return modules;
  }

  @Override
  public void setBindings() {
    // SS we need to test
    String odometryName = VisionPoseEstimator.class.getSimpleName(); // or novision "odometry"
    OdometryInterface odo = RobotContainer.getSubsystemOrNull(odometryName);
    DriveTrainInterface sdt = RobotContainer.getSubsystemOrNull("drivetrain");
    HID_Subsystem dc = RobotContainer.getSubsystem("DC");

    // Initialize PathPlanner, if we have needed Subsystems
    if (odo != null && sdt != null) {
      AutoPPConfigure.configureAutoBuilder(sdt, odo);
      PathfindingCommand.warmupCommand().schedule();
    }
    
    // Competition bindings -  NOTE: OPR portion of comp binding disabled 
    // until done with integration.
    // BindingsCompetition.ConfigureCompetition(dc, false);
    
    // Place your test binding in ./testBinding/<yourFile>.java and call it here
    // comment out any conflicting bindings. Try not to push with your bindings
    // active. Just comment them out.
    
    DPLPathTest.myBindings(dc); 
    // ElevTest.myBindings(dc);
    // EndEffectorTest.myBindings(dc);
    // GITest.myBindings(dc);

    // FOR BOT ON BOARD you can configure bindings directly here
    // or create a binding file in ./testBindings/BotOnBoard<N>.java

    // Anything else that needs to run after binding/commands are created
    VisionPoseEstimator vpe = RobotContainer.getSubsystemOrNull(VisionPoseEstimator.class);
    if (vpe != null) 
      vpe.configureGyroCallback();
    
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  @Override
  public boolean burnFlash() {
    return true;
  }

  SendableChooser<Command> autoChooser;

  @Override
  public void setupRegisteredCommands() {
    RegisteredCommandsTest.RegisterCommands(); 

    //enable chooser - builds autochooser list
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);   
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

  /*
   * Add additional calls to the robotPeriodic loop
   */
  @Override
  public void periodic() {
    UXTrim.periodic();
  }


}