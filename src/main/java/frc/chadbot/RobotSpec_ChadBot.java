package frc.chadbot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static frc.lib2202.Constants.MperFT;

import frc.chadbot.Constants.CAN;
import frc.chadbot.bindings.Comp_ChadBot;
import frc.chadbot.subsystems.Intake_Subsystem;
import frc.chadbot.subsystems.Magazine_Subsystem;
import frc.chadbot.subsystems.Positioner_Subsystem;
//Note there is a sensors in lib2202, but we want the robot specific one
import frc.chadbot.subsystems.Sensors_Subsystem;
import frc.chadbot.subsystems.shooter.Shooter_Subsystem;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
import frc.lib2202.command.swerve.FieldCentricDrive;
import frc.lib2202.subsystem.Limelight;
import frc.lib2202.subsystem.Odometry;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig.CornerID;


public class RobotSpec_ChadBot implements IRobotSpec {

  // In debugger window set env with:  $env:serialnum='03238151'
  // Chad's subsystems and objects
  final SubsystemConfig ssConfig = new SubsystemConfig("ChadBot", "03238151")
      .addAlias(Sensors_Subsystem.class, "sensors")
      .addAlias(Limelight.class,"limelight")
      .addAlias(SwerveDrivetrain.class,"drivetrain") // must be after Sensors
      .add(HID_Subsystem.class, "DC", () -> {
        return new HID_Subsystem(0.3, 0.9, 0.05);
      })
      //TODO update to VPE when in lib2202
      .add(OdometryInterface.class, "odometry", () -> {
          var obj = new Odometry();
          //obj.new OdometryWatcher();
          return obj;
      })
      //rest of Chad's subsystems
      .add(Shooter_Subsystem.class)
      .add(Intake_Subsystem.class)
      .add(Magazine_Subsystem.class)
      .add(Positioner_Subsystem.class);

  boolean swerve = true;

  // Robot Speed Limits
  RobotLimits robotLimits = new RobotLimits(FeetPerSecond.of(15.0), DegreesPerSecond.of(180.0));

  // Chassis
  double kWheelCorrectionFactor = .995;
  double kSteeringGR = 12.8;
  double kDriveGR = 8.14;
  double kWheelDiameter = MperFT * 4.0 / 12.0; // [m]

  final ChassisConfig chassisConfig = new ChassisConfig(
      MperFT * (21.516 / 12.0) / 2.0, // X offset
      MperFT * (24.87 / 12.0) / 2.0, // Y offset
      kWheelCorrectionFactor,
      kWheelDiameter,
      kSteeringGR,
      kDriveGR);

  public RobotSpec_ChadBot() {
    // finish the config by adding this spec to the ssConfig
    ssConfig.setRobotSpec(this);
  }

  public SubsystemConfig getSubsystemConfig(){
    return ssConfig;
  }

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
    ModuleConfig[] modules = new ModuleConfig[4];
    modules[CornerID.FrontLeft.getIdx()] = new ModuleConfig(CornerID.FrontLeft,
        CAN.DT_FL_CANCODER, CAN.DT_FL_DRIVE, CAN.DT_FL_ANGLE,
        3.764)
        .setInversions(true, false, false);// drive was false

    modules[CornerID.FrontRight.getIdx()] = new ModuleConfig(CornerID.FrontRight,
        CAN.DT_FR_CANCODER, CAN.DT_FR_DRIVE, CAN.DT_FR_ANGLE, // 30, 26, 27,
        17.682)
        .setInversions(false, false, false); // drive was true

    modules[CornerID.BackLeft.getIdx()] = new ModuleConfig(CornerID.BackLeft,
        CAN.DT_BL_CANCODER, CAN.DT_BL_DRIVE, CAN.DT_BL_ANGLE, // 28, 22, 23,
        63.31)
        .setInversions(true, false, false); //was false

    modules[CornerID.BackRight.getIdx()] = new ModuleConfig(CornerID.BackRight,
        CAN.DT_BR_CANCODER, CAN.DT_BR_DRIVE, CAN.DT_BR_ANGLE, // 31, 24, 25,
        -20.912)
        .setInversions(false, false, false); //was true

    return modules;
  }

  @Override
  public void setBindings() {
    HID_Subsystem dc = RobotContainer.getSubsystem("DC");
    Comp_ChadBot.ConfigureCompetition(dc);
    // BindingsOther.ConfigureOther(dc);

  }


  @Override
    public void setDefaultCommands() {
       SwerveDrivetrain drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
        if (drivetrain != null) {
            drivetrain.setDefaultCommand(new FieldCentricDrive());
          }
    }


}
