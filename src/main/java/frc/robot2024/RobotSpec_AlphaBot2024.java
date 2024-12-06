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
import frc.lib2202.command.swerve.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.FieldCentricDrive;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.subsystem.Limelight;
import frc.lib2202.subsystem.hid.HID_Xbox_Subsystem;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig.CornerID;
import frc.lib2202.util.PIDFController;
import frc.robot2024.Constants.CAN;
import frc.robot2024.subsystems.sensors.Sensors_Subsystem;

public class RobotSpec_AlphaBot2024 implements IRobotSpec {
  // Subsystems and other hardware on 2024 Robot
  final SubsystemConfig ssconfig = new SubsystemConfig("CompetitionBotAlpha2024", "032381BF")
      // deferred construction via Supplier<Object> lambda
      .add(PowerDistribution.class, "PDP", () -> {
        var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
        pdp.clearStickyFaults();
        return pdp;
      })
      //.add(PneumaticsControl.class)
      // .add(BlinkyLights.class, "LIGHTS")
      .add(HID_Xbox_Subsystem.class, "DC", () -> {
        return new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
      })
      .add(Sensors_Subsystem.class)
      .add(Limelight.class)
      .add(SwerveDrivetrain.class); // must be after LL and Sensors
      
      // .add(Command.class, "DT_Monitor", () -> {return new DTMonitorCmd();})
      
      /*  ALPHABOT PARTIALLY DISSAMBLED
      .add(Intake.class)
      .add(Command.class, "IntakeWatcher", () -> {
        return RobotContainer.getSubsystem(Intake.class).getWatcher();
      })
      .add(Shooter.class)
      .add(Command.class, "ShooterWatcher", () -> {
        // cast to get the correct type of shooter
        return (RobotContainer.getSubsystem(Shooter.class)).getWatcher();
      })
      .add(Transfer.class)
      .add(Command.class, "TransferWatcher", () -> {
        return RobotContainer.getSubsystem(Transfer.class).getWatcher();
      });
      */
      

  // set this true at least once after robot hw stabilizes
  boolean burnFlash = false;
  boolean swerve = true;

  // Robot Speed Limits
  double maxSpeed = 15.0 * MperFT; // [m/s]
  double maxRotationRate = 180.0;  // [deg/s]
  RobotLimits robotLimits = new RobotLimits(maxSpeed, maxRotationRate);

  // Chassis
  double kWheelCorrectionFactor = .957;
  double kSteeringGR = 21.428;
  double kDriveGR = 6.12;
  double kWheelDiameter = MperFT * 4.0 / 12.0; // [m]

  final ChassisConfig chassisConfig = new ChassisConfig(
      MperFT * (25.0 / 12.0) / 2.0, // x
      MperFT * (20.75 / 12.0) / 2.0, // y
      kWheelCorrectionFactor, // scale [] <= 1.0
      kWheelDiameter,
      kSteeringGR,
      kDriveGR,
      new PIDFController(0.085, 0.00055, 0.0, 0.21292), // drive
      new PIDFController(0.01, 0.0, 0.0, 0.0) // angle
  );

  public RobotSpec_AlphaBot2024() {
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
    return RobotContainer.getSubsystem(Sensors_Subsystem.class);
  }

  @Override
  public ChassisConfig getChassisConfig() {
    return chassisConfig;
  }

  @Override
  public ModuleConfig[] getModuleConfigs() {
    // from original constants Alpha and Beta (comp) have same CAN ID for swerve
    // comp2024AlphaBotOffsets = new WheelOffsets(43.85746387, 24.096825, -65.21481,
    // -43.066333125); //FL BL FR BR
    // final ModuleConfig comp2024CAN_FL = new ModuleConfig(29, 24, 25);
    // final ModuleConfig comp2024CAN_FR = new ModuleConfig(30, 26, 27);
    // final ModuleConfig comp2024CAN_BL = new ModuleConfig(28, 22, 23);
    // final ModuleConfig comp2024CAN_BR = new ModuleConfig(31, 20, 21);
    // final CANConfig comp2024BotCANConfig = new CANConfig(comp2024CAN_FL,
    // comp2024CAN_FR, comp2024CAN_BL, comp2024CAN_BR);

    // public static final ChassisInversionSpecs comp2024BotAlphaInversionSpecs =
    // new ChassisInversionSpecs(
    // new ModuleInversionSpecs(true, true, false), // FR
    // new ModuleInversionSpecs(false, true, false), // FL
    // new ModuleInversionSpecs(true, true, false), // BR
    // new ModuleInversionSpecs(false, true, false)); // BL
    
    /*================OffsetDebug==================
    FrontLeft: offset=0.0, internal=45.615234375 cancoder_measured=45.52734375 , if wheel zero-aligned adjust offset by -45.52734375
    FrontRight: offset=0.0, internal=-60.468753814697266 cancoder_measured=-60.46875 , if wheel zero-aligned adjust offset by 60.46875
    BackLeft: offset=0.0, internal=22.587890625 cancoder_measured=22.587890625 , if wheel zero-aligned adjust offset by -22.587890625
    BackRight: offset=0.0, internal=-42.01171875 cancoder_measured=-41.923828125 , if wheel zero-aligned adjust offset by 41.923828125
    ============OffsetDebug Done============== */

    ModuleConfig[] modules = new ModuleConfig[4];
    modules[CornerID.FrontLeft.getIdx()] = new ModuleConfig(CornerID.FrontLeft,
        29, 24, 25,
        -45.52734375) //43.85746387)
        .setInversions(false, true, false);

    modules[CornerID.FrontRight.getIdx()] = new ModuleConfig(CornerID.FrontRight,
        30, 26, 27,
        60.46875)//-65.21481)
        .setInversions(true, true, false);

    modules[CornerID.BackLeft.getIdx()] = new ModuleConfig(CornerID.BackLeft,
        28, 22, 23,
       -22.587890625)// 24.096825)
        .setInversions(false, true, false);

    modules[CornerID.BackRight.getIdx()] = new ModuleConfig(CornerID.BackRight,
        31, 20, 21,
       41.923828125)// -43.066333125)
        .setInversions(true, true, false);

    return modules;
  }

  @Override
  public void setBindings() {
    HID_Xbox_Subsystem dc = RobotContainer.getSubsystem("DC");
    // pick one of the next two lines
    // cant use - parts removed from bot
    // BindingsCompetition.ConfigureCompetition(dc);

    var driver = dc.Driver();
    //var drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);

    // Driver buttons
    driver.leftTrigger().whileTrue(new FieldCentricDrive());
    driver.y().onTrue(new AllianceAwareGyroReset(true));
    //driver.rightTrigger().whileTrue(new TargetCentricDrive(Tag_Pose.ID4, Tag_Pose.ID7));

  }

  @Override
  public boolean burnFlash() {
    return burnFlash;
  }

  @Override
  public SendableChooser<Command> getRegisteredCommands() {
    // no robot parts to support thse now
    //return RegisteredCommands.RegisterCommands();
    return null;
  }

  @Override
    public void setDefaultCommands() {
       SwerveDrivetrain drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
        if (drivetrain != null) {
            drivetrain.setDefaultCommand(new RobotCentricDrive());
          }
    }


}