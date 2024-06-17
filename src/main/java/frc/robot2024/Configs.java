package frc.robot2024;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2024.Constants.CAN;
import frc.robot2024.commands.Shooter.DistanceInterpretor;
import frc.robot2024.subsystems.AmpMechanism;
import frc.lib2202.subsystem.BlinkyLights;
import frc.lib2202.subsystem.Limelight;
import frc.robot2024.subsystems.Climber;
import frc.robot2024.subsystems.Intake;
import frc.robot2024.subsystems.PneumaticsControl;
import frc.robot2024.subsystems.Shooter;
import frc.robot2024.subsystems.ShooterServo;
import frc.robot2024.subsystems.Transfer;
import frc.robot2024.subsystems.Sensors.Sensors_Subsystem;
import frc.lib2202.subsystem.swerve.DTMonitorCmd;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.lib2202.subsystem.hid.HID_Xbox_Subsystem;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.SubsystemConfig;

/*
 * The Subsystems and object in Configs will be created by RobotContainer 
 * when the robot's serial number is read by RobotSpecs. 
 * 
 * All construction is deferred until inside RobotContainer so be careful
 * with any 'new' operators, they should be wrapped in a lambda. For an example
 * see PDP below.
 */
public class Configs {

  // Subsystems and other hardware on 2024 Robot
  public static final SubsystemConfig comp2024AlphaBotSubsystemConfig = new SubsystemConfig()
      // deferred construction via Supplier<Object> lambda
      .add(PowerDistribution.class, "PDP", () -> {
        var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
        pdp.clearStickyFaults();
        return pdp;
      })
      .add(PneumaticsControl.class)
      // .add(BlinkyLights.class, "LIGHTS")
      .add(HID_Xbox_Subsystem.class, "DC", () -> {
        return new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
      })
      .add(Sensors_Subsystem.class)
      .add(Limelight.class)
      .add(SwerveDrivetrain.class) // must be after LL and Sensors
      // .add(Command.class, "DT_Monitor", () -> {return new DTMonitorCmd();})
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

  public static final SubsystemConfig comp2024BetaBotSubsystemConfig = new SubsystemConfig()
      // deferred construction via Supplier<Object> lambda
      .add(PowerDistribution.class, "PDP", () -> {
        var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
        pdp.clearStickyFaults();
        return pdp;
      })
      .add(BlinkyLights.class, "LIGHTS")
      .add(HID_Xbox_Subsystem.class, "DC", () -> {
        return new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
      })
      .add(Sensors_Subsystem.class)
      .add(Limelight.class)
      .add(SwerveDrivetrain.class) // must be after LL and Sensors
      .add(Command.class, "DT_Monitor", () -> {return new DTMonitorCmd();})
      .add(Intake.class)
      .add(Command.class, "IntakeWatcher", () -> {
        return RobotContainer.getSubsystem(Intake.class).getWatcher();
      })
      .add(Transfer.class)
      .add(ShooterServo.class)
      .add(Climber.class)
      .add(AmpMechanism.class)
      .add(Command.class, "ClimberWatcher", () -> {
        return RobotContainer.getSubsystem(Climber.class).getWatcher();
      })
      .add(Command.class, "ShooterServoWatcher", () -> {
        // cast to get the correct type of shooter
        return (RobotContainer.getSubsystem(ShooterServo.class)).getWatcher();
      })
      .add(Command.class, "TransferWatcher", () -> {
        return RobotContainer.getSubsystem(Transfer.class).getWatcher();
      })
      .add(DistanceInterpretor.class, "TargetingTable", () ->{ return DistanceInterpretor.getSingleton();})
      ;

  // Subsystems and hardware on Tim 2.0
  public static final SubsystemConfig swerveBotSubsystemConfig = new SubsystemConfig()
      .add(Sensors_Subsystem.class)
      .add(Limelight.class)
      .add(SwerveDrivetrain.class)
      .add(HID_Xbox_Subsystem.class, "DC", () -> {
        return new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
      });

  // Chad's subsystems and objects
  public static final SubsystemConfig chadBotSubsystemConfig = new SubsystemConfig()
      .add(Sensors_Subsystem.class)
      .add(Limelight.class)
      .add(SwerveDrivetrain.class) // must be after LL and Sensors
      .add(HID_Xbox_Subsystem.class, "DC", () -> {
        return new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
      });

  public static final SubsystemConfig doofBotSubsystemConfig = new SubsystemConfig()
      .add(Sensors_Subsystem.class)
      .add(Limelight.class)
      .add(HID_Xbox_Subsystem.class, "DC", () -> {
        return new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
      })
      .add(SwerveDrivetrain.class);

  public static final SubsystemConfig botOnBoardSystemConfig = new SubsystemConfig()
  // .add(null) useful stuff for bot on boards
  ;
}
