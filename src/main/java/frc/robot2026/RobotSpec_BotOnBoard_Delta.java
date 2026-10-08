package frc.robot2026;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.robot2026.Constants.CAN;
import frc.robot2026.subsystems.Shooter.Shooter;

public class RobotSpec_BotOnBoard_Delta implements IRobotSpec {

  //Bot On Board Delta
  // $env:serialnum = "3061025"
  final SubsystemConfig ssconfig = new SubsystemConfig("BotOnBoard_Delta", "03061025")
      // Add the subsystems or components use by this Bot-on-Board    
      // Bot-On-Board can always use controlers for test binding
       .add(HID_Subsystem.class, "DC", () -> {
        return new HID_Subsystem(0.3, 0.9, 0.05);
      })
      .add(PowerDistribution.class, "PDP", () -> {
        var pdp = new PowerDistribution(CAN.PDP, ModuleType.kCTRE);
        pdp.clearStickyFaults();
        return pdp;
      })
      // .add(Intake.class)
      .add(Shooter.class, "ShooterRight", () ->{
        return new Shooter("ctre", CAN.ShooterIDRight, false); //opts: rev,ctre,multi
      })
      .add(Shooter.class, "ShooterLeft", () -> {
        return new Shooter("ctre", CAN.ShooterIDLeft);
      })
      ;


  // Robot Speed Limits
  RobotLimits robotLimits = new RobotLimits(FeetPerSecond.of(15.0), DegreesPerSecond.of(180.0));

  public RobotSpec_BotOnBoard_Delta() {
    // add the specs to the ssconfig
    ssconfig.setRobotSpec(this);
  }

  // Required method that use the specs above

  @Override
  public RobotLimits getRobotLimits() {
    return robotLimits;
  }

  @Override
  public void setBindings() {
    HID_Subsystem dc = RobotContainer.getSubsystemOrNull("DC");
    if (dc == null ){
        // BOB doesn't need DC. Return early if DC doesn't exist
        System.out.println("Warning: DC doesn't exist not setting bindings");
      return;
    }

    if (dc.Driver() instanceof CommandPS4Controller) {
      // CommandPS4Controller operator = (CommandPS4Controller)dc.Driver();
    } else {
      @SuppressWarnings("unused")
      CommandXboxController driver = (CommandXboxController)dc.Driver();
      CommandXboxController operator = (CommandXboxController)dc.Operator();

      // TEST BINDING FOR NOW 
      Shooter shooter = RobotContainer.getSubsystem(Shooter.class);
      shooter.setTestBindings(operator);  // uses triggers
    }  
    // show what cmds are running
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  SendableChooser<Command> autoChooser;

  @Override
  public void setupRegisteredCommands() {
  
  }
  
  @Override
  public SendableChooser<Command> getChooser() { 
    return autoChooser;  //this is null, unlesse we setup autoChooser
  }

  @Override
  public void setDefaultCommands() {
   
  }
}