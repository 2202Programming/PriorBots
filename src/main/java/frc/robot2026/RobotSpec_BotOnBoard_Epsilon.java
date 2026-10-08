package frc.robot2026;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;

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
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.robot2026.Constants.CAN;
import frc.robot2026.subsystems.Shooter.Indexer;

public class RobotSpec_BotOnBoard_Epsilon implements IRobotSpec {

  //Bot On Board Epsilon
  // $env:serialnum = "0326F275"
  final SubsystemConfig ssconfig = new SubsystemConfig("BotOnBoard_Epsilon", "0326F275")
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
      .add(Indexer.class)
      ;
      

  // Robot Speed Limits
  RobotLimits robotLimits = new RobotLimits(FeetPerSecond.of(15.0), DegreesPerSecond.of(180.0));

  public RobotSpec_BotOnBoard_Epsilon() {
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
    @SuppressWarnings("unused")
    CommandXboxController driver = (CommandXboxController)dc.Driver();
    CommandXboxController operator = (CommandXboxController)dc.Operator();

    // TEST BINDING FOR NOW 
    Indexer hopper = RobotContainer.getSubsystem(Indexer.class);
    hopper.setTestBindings(operator);  // uses triggers
    // operator.a().whileTrue(new IntakePwrSpin(0.2));

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