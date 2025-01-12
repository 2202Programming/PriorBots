package frc.robot2024;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static frc.lib2202.Constants.MperFT;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot2024.subsystems.sensors.Sensors_Subsystem;

public class RobotSpec_ChadBot implements IRobotSpec {

  // Chad's subsystems and objects
  final SubsystemConfig ssConfig = new SubsystemConfig("ChadBot", "03238151")
      .add(Sensors_Subsystem.class)
      .add(Limelight.class)
      .add(SwerveDrivetrain.class) // must be after LL and Sensors
      .add(HID_Xbox_Subsystem.class, "DC", () -> {
        return new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
      });

  // set this true at least once after robot hw stabilizes
  boolean burnFlash = false;
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
      MperFT * (24.87 / 12) / 2.0, // Y offset
      kWheelCorrectionFactor,
      kWheelDiameter,
      kSteeringGR,
      kDriveGR);

  public RobotSpec_ChadBot() {
    // finish the config by adding this spec to the ssConfig
    ssConfig.setRobotSpec(this);
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
        7, 20, 21,
        -175.60)
        .setInversions(false, true, false);

    modules[CornerID.FrontRight.getIdx()] = new ModuleConfig(CornerID.FrontRight,
        30, 26, 27,
        -162.15)
        .setInversions(true, false, false);

    modules[CornerID.BackLeft.getIdx()] = new ModuleConfig(CornerID.BackLeft,
        28, 22, 23,
        -115.40)
        .setInversions(false, false, false);

    modules[CornerID.BackRight.getIdx()] = new ModuleConfig(CornerID.BackRight,
        31, 24, 25,
        158.81)
        .setInversions(true, false, false);

    return modules;
  }

  @Override
  public void setBindings() {
    HID_Xbox_Subsystem dc = RobotContainer.getSubsystem("DC");
    // pick one of the next two lines
    //TODO - chadbot should have own comp bindings
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
