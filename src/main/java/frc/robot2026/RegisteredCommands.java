package frc.robot2026;

import javax.net.ssl.TrustManager;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.FaceToTag;
import frc.lib2202.command.swerve.RotateTo;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.robot2026.command.AgitateOS;
import frc.robot2026.command.AgitateV2;
import frc.robot2026.command.autoClimberCommand;
import frc.robot2026.command.shooter.AutoShoot;
import frc.robot2026.subsystems.Climber;
import frc.robot2026.subsystems.Hopper;
import frc.robot2026.subsystems.Intake;
import frc.robot2026.subsystems.Shooter.Indexer;
import frc.robot2026.subsystems.Shooter.Shooter;
import frc.robot2026.subsystems.Shooter.Targeter;

/*
 * Place commands named in PathPlaner autos here.
 */
@SuppressWarnings("unused")
public class RegisteredCommands {
    static DriveTrainInterface drivetrain;
    static HID_Subsystem dc;
    static Climber climber;
    static Shooter shooter_left;
    static Shooter shooter_right;
    static Indexer indexer_left;
    static Indexer indexer_right;
    static Hopper hopper;
    static Intake intake;
    static Targeter targeter;
    
    static void get_references(){
        //subsystem refs for building registerd commands
        shooter_left = RobotContainer.getSubsystem("shooter_left");
        shooter_right = RobotContainer.getSubsystem("shooter_right");
        drivetrain = RobotContainer.getSubsystem("drivetrain");  
        indexer_left = RobotContainer.getSubsystem("indexer_left");
        indexer_right = RobotContainer.getSubsystem("indexer_right");
        intake = RobotContainer.getSubsystem("intake");
        climber = RobotContainer.getSubsystem("climber");
        hopper = RobotContainer.getSubsystem(Hopper.class); 
        targeter = RobotContainer.getSubsystem(Targeter.class);
    }


    // Named Command Factories
    public static Command ncShoot() {
        get_references();
        
        //final double face_timeout = 2.0;   //pathing should leave us close
        final double agitate_period = .25;
        final double belts_speed = 0.65;
        final double agitate_delay = 0.25;
        final double agitate_in_spd = 0.7;
        var cmd = new SequentialCommandGroup(
                new PrintCommand("Shooting lots of fuel ..."),                            
                // not working consistently, needs more testing   
                // new RotateTo(targeter.getRedHub(), targeter.getBlueHub(), face_timeout).setP(8.0),
                // the commands in this parallel group DO NOT FINISH ...
                new ParallelRaceGroup(
                        // new AgitateV2().repeatedly(),
                        new AgitateOS(true, belts_speed, agitate_period, agitate_delay, agitate_in_spd),
                        new AutoShoot("left", targeter::getTargetSpeed, targeter::getTolerance, 1.0),
                        new AutoShoot("right", targeter::getTargetSpeed, targeter::getTolerance, 1.0)
                ).withTimeout(6.0),
                new PrintCommand("                     ... nothing but net.")
        );         
        cmd.setName("ncShoot");
        return cmd;
    }

    static Command ncSpinup() {
        Command cmd =     new InstantCommand( () -> {
            shooter_left.flywheel.setSetpoint(SPINUP);
            shooter_right.flywheel.setSetpoint(SPINUP);
       });
       
       return cmd;
    }

    static double SPINUP = 17.65; // about 10ft
    public static void RegisterCommands() {    
        get_references();

        // Construct all the commands and register them to NamedCommands for PathPlanner
        NamedCommands.registerCommand("shoot", ncShoot());
        NamedCommands.registerCommand("intake_on", intake.cmdRunWhileFuel(.45, .5));
        NamedCommands.registerCommand("climb_right", new autoClimberCommand(false));
        NamedCommands.registerCommand("climb_left", new autoClimberCommand(true));
        NamedCommands.registerCommand("noise",   new PrintCommand("noise ... --- ..."));

        // Event Markers set in paths need an EventTrigger to execute the command.
        new EventTrigger("spinup").onTrue(ncSpinup());

    }
}