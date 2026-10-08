package frc.robot2026;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static frc.lib2202.Constants.MperFT;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.lib2202.util.PIDFController;
import frc.robot2026.Constants.CAN;
import frc.robot2026.subsystems.Shooter.Indexer;
import frc.robot2026.subsystems.Shooter.Shooter;

public class RobotSpec_BotOnBoard_Zeta implements IRobotSpec {
  // $env:serialnum = "0312db1a"
  final SubsystemConfig ssconfig = new SubsystemConfig("BotOnBoard-Zeta", "0312db1a")
      // deferred construction via Supplier<Object> lambda
      .add(PowerDistribution.class, "PDP", () -> {
        var pdp = new PowerDistribution(CAN.PDP, ModuleType.kCTRE);
        pdp.clearStickyFaults();
        return pdp;
      })

      .add(HID_Subsystem.class, "DC", () -> {
        return new HID_Subsystem(0.3, 0.9, 0.05);

      })
      // .add(Intake.class)
      .add(Shooter.class, "ShooterRight", () ->{
        return new Shooter("ctre", CAN.ShooterIDRight, false); //opts: rev,ctre,multi
      })
      .add(Shooter.class, "ShooterLeft", () -> {
        return new Shooter("ctre", CAN.ShooterIDLeft);
      })
      .add(Indexer.class)
      ;
      // below are optional watchers for shuffeleboard data - disable if need too.

  // set this true at least once after robot hw stabilizes
  boolean burnFlash = false;
  boolean swerve = true;

  // Robot Speed Limits
  RobotLimits robotLimits = new RobotLimits(FeetPerSecond.of(15.0), DegreesPerSecond.of(180.0));

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

  public RobotSpec_BotOnBoard_Zeta() {
    // finish BetaBot's drivePIDF
    // add the specs to the ssconfig
    ssconfig.setRobotSpec(this);
  }

  // Required method that use the specs above

  @Override
  public RobotLimits getRobotLimits() {
    return robotLimits;
  }

  @Override
  public ChassisConfig getChassisConfig() {
    return chassisConfig;
  }

  @Override
  public ModuleConfig[] getModuleConfigs() {
    // bot on board, no modules
    ModuleConfig[] modules = null;
    return modules;
  }

  @Override
  public void setBindings() {
    HID_Subsystem dc = RobotContainer.getSubsystem("DC");
    if (dc.Driver() instanceof CommandPS4Controller) {
      // CommandPS4Controller operator = (CommandPS4Controller)dc.Driver();
    } else {
      @SuppressWarnings("unused")
      CommandXboxController driver = (CommandXboxController)dc.Driver();
      CommandXboxController operator = (CommandXboxController)dc.Operator();

      // TEST BINDING FOR NOW 
       Shooter shooter = RobotContainer.getSubsystemOrNull("Shooter");
      if (shooter!=null) {
        shooter.setTestBindings(driver);
      } 

      Indexer hopper = RobotContainer.getSubsystemOrNull(Indexer.class);
      if (hopper !=null) {
        hopper.setTestBindings(operator);  // uses triggers
      }
      // operator.a().whileTrue(new IntakePwrSpin(0.2));
    }
  }

  @Override
  public void setDefaultCommands() {
  }

}