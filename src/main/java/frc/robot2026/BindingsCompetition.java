package frc.robot2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.ScaleDriver;
import frc.lib2202.command.pathing.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.command.swerve.RotateTo;
import frc.lib2202.command.swerve.StrafeDrive;
import frc.lib2202.command.swerve.TargetCentricDrive;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.hid.TMJoystickController;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.robot2026.command.Agitate;
import frc.robot2026.command.AgitateOS;
import frc.robot2026.command.Climb2;
import frc.robot2026.command.autoClimberCommand;
import frc.robot2026.command.shooter.AutoShoot;
import frc.robot2026.subsystems.Climber;
import frc.robot2026.subsystems.Hopper;
import frc.robot2026.subsystems.Intake;
import frc.robot2026.subsystems.Shooter.Indexer;
import frc.robot2026.subsystems.Shooter.Shooter;
import frc.robot2026.subsystems.Shooter.Targeter; 

import frc.robot2026.RegisteredCommands;

/*
 * Please don't edit this without leads/mentor/driveteam review
 */
@SuppressWarnings("unused")
public final class BindingsCompetition {
    // subsystem references for use in command bindings
    public static DriveTrainInterface drivetrain;
    public static HID_Subsystem dc;
    public static Climber climber;
    public static Shooter shooter_left;
    public static Shooter shooter_right;
    public static Indexer indexer_left;
    public static Indexer indexer_right;
    public static Hopper hopper;
    public static Intake intake;
    public static Targeter targeter;

    private static void get_references() {
        // Subsystems must exist in RobotSpec, if they don't an NPE is thrown.
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

    public static void ConfigureCompetition(HID_Subsystem dc) {
        ConfigureCompetition(dc, true);
    }

    // optional disable opr binding for testing
    public static void ConfigureCompetition(HID_Subsystem _dc, boolean initOpr) {
        // get references for the commands to use
        dc = _dc;
        get_references();
        DriverBinding();
        if (initOpr) 
            OperatorBindings();     

        //Auto intake, works well if lightgate stays aligned.
        //This seems to ignore kCancelIncomming - converted to default Command for intake
        //Trigger HasFuel = new Trigger(intake::hasFuel);
        //HasFuel.onTrue(intake.cmdRunWhileFuel(.45, 0.5) );  //changed from .8 to 0.65 by drive team on 2/28
    }

    private static void DriverBinding() {
        var generic_driver = dc.Driver();

        // Driver Buttons depend on the type of controller drivers selects
        if (generic_driver instanceof TMJoystickController) {
            // Joystick
            TMJoystickController joystick = (TMJoystickController) generic_driver;

        } else if (generic_driver instanceof CommandXboxController) {
            // XBox
            CommandXboxController driver = (CommandXboxController) generic_driver;
            driver.rightBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));
            driver.povLeft().whileTrue(new TargetCentricDrive(targeter.getRedHub(), targeter.getBlueHub()) 
                                    .setP(4.0));            
            // testing on rotate to target
            driver.povRight().onTrue(new RotateTo(BindingsCompetition.targeter.getRedHub(),
                                               BindingsCompetition.targeter.getBlueHub(),1.0)
                                               .setP(4.0));

            driver.y().onTrue(new AllianceAwareGyroReset());

            // Strafe based on Vy motion. Command is Alliance aware.
            driver.povUp().whileTrue(new StrafeDrive(135.0));           
            driver.povDown().whileTrue(new StrafeDrive(45.0));

            // Driver will wants precision robot-centric throttle drive on left bumper
            driver.leftBumper().whileTrue(new ParallelCommandGroup(
                    new ScaleDriver(0.3),
                    new RobotCentricDrive(drivetrain, dc)));

            // Shoot with targetSpeed based on distance to hub - moved to Opr.B()
            // driver.leftTrigger(0.7).whileTrue(new AutoShoot("left", targeter::getTargetSpeed, targeter::getTolerance, 1.0));
            // driver.leftTrigger(0.7).whileTrue(new AutoShoot("right", targeter::getTargetSpeed, targeter::getTolerance, 1.0));
            // driver.leftTrigger(0.1).whileTrue(hopper.cmdBeltPct(0.705))
            //         .onFalse(hopper.cmdBeltPct(0));

            // Driver wants to manually fire/pass
            //driver.rightTrigger(0.7).whileTrue(new AutoShoot("left", targeter::getManualSpeed, targeter::getManualTolerance, 1.0));
            //driver.rightTrigger(0.7).whileTrue(new AutoShoot("right", targeter::getManualSpeed, targeter::getManualTolerance, 1.0));
            //driver.rightTrigger(0.1).whileTrue(hopper.cmdBeltPct(0.705))
            //        .onFalse(hopper.cmdBeltPct(0).withName("rtTrig_hopperZero*****"));

            //Strafe commands on triggers
            driver.leftTrigger(0.5).whileTrue(new StrafeDrive(45.0));
            driver.rightTrigger(0.5).whileTrue(new StrafeDrive(135.0));    

        } else {
            DriverStation.reportError("Comp Bindings: No driver bindings set, check controllers.", false);
        }
    }

    static void OperatorBindings() {
        var sideboard = dc.SwitchBoard();
        var generic_opr = dc.Operator();

        Trigger Cal = sideboard.sw11(); // calibration button (conventional)
        Trigger NotCal = Cal.negate(); // regular competition mode
        Trigger DumbShooter = sideboard.sw26(); // placeholder for fallback to fixed shooting region

        // buttons depend on what controller is plugged in
        if (generic_opr instanceof CommandXboxController) {
            CommandXboxController operator = (CommandXboxController) generic_opr;

            // intake bindings
            sideboard.sw21().onTrue(targeter.OverrideTargetDistanceFT(9.99))   // fixed distance
                            .onFalse(targeter.OverrideTargetDistanceFT(0.0));  //use vision distance

            // intake / hopper in
            operator.leftBumper().whileTrue(hopper.cmdBeltPct(0.705))
                    .onFalse(hopper.cmdBeltPct(0));
            operator.leftBumper().whileTrue(intake.cmdPctPwr(0.65).repeatedly())
                    .onFalse(intake.cmdPctPwr(0.0));

            /* === Intake & Hopper Eject ===
            * The .repeatedly() decorator wraps around the cmdPctPwr() instant command so that
            * whenever incoming commands using the intake are sceduled, they are ignored.
            * This prevents the auto intake lightgate trigger command from being scheduled
            * when the balls being ejected. */ 
            operator.rightBumper().whileTrue(hopper.cmdBeltPct(-1))
                    .onFalse(hopper.cmdBeltPct(0));
            operator.rightBumper().whileTrue(intake.cmdEject(-1.0))
                    .onFalse(intake.cmdPctPwr(0.0));

            // shooter unblock    
            operator.y().whileTrue(shooter_left.cmdVelocity(-15))
                        .whileTrue(shooter_right.cmdVelocity(-15))
                        .onFalse(shooter_left.cmdVelocity(0))
                        .onFalse(shooter_right.cmdVelocity(0));
                        
            operator.y().whileTrue(indexer_left.cmdSetPct(-1))
                        .whileTrue(indexer_right.cmdSetPct(-1))
                        .onFalse(indexer_left.cmdSetPct(0))
                        .onFalse(indexer_right.cmdSetPct(0));

            //agitate back and forth, uses intake trigger to not lose fuel
            // operator.a().whileTrue(new RepeatCommand(new Agitate()) ) 
            //             .onFalse(hopper.cmdBeltPct(0)) 
            //             .onFalse(intake.cmdPctPwr(0));
                        
            operator.a().whileTrue(new AgitateOS(true, 0.655, .25, 0.25, .7));

            double spinUp = 17.501;
            operator.b().onTrue(shooter_left.cmdVelocity(spinUp))
                        .onFalse(shooter_left.cmdVelocity(0.0));
            operator.b().onTrue(shooter_right.cmdVelocity(spinUp))
                        .onFalse(shooter_right.cmdVelocity(0.0));
                            
            // Shoot with targetSpeed based on distance to hub - moved to Opr.B()
            operator.leftTrigger(0.7).whileTrue(new AutoShoot("left", targeter::getTargetSpeed, targeter::getTolerance, 1.0));
            operator.leftTrigger(0.7).whileTrue(new AutoShoot("right", targeter::getTargetSpeed, targeter::getTolerance, 1.0));
            operator.leftTrigger(0.1).whileTrue(hopper.cmdBeltPct(0.705))
                    .onFalse(hopper.cmdBeltPct(0));

            // Driver wants to manually fire/pass
            operator.rightTrigger(0.7).whileTrue(new AutoShoot("left", targeter::getManualSpeed, targeter::getManualTolerance, 1.0));
            operator.rightTrigger(0.7).whileTrue(new AutoShoot("right", targeter::getManualSpeed, targeter::getManualTolerance, 1.0));
            operator.rightTrigger(0.1).whileTrue(hopper.cmdBeltPct(0.705))
                    .onFalse(hopper.cmdBeltPct(0));

            // TESTING for now
            //operator.back().onTrue(new autoClimberCommand(true)); // LEFT
            //operator.start().onTrue(new autoClimberCommand(false)); // RIGHT
         
            // Calibration Commands
            Cal.and(sideboard.sw12()).whileTrue(climber.setVelocityCmd(Climber.ClimbCalibrateVel))
                    .onFalse(climber.setVelocityCmd(0.0));
            Cal.and(sideboard.sw13()).whileTrue(climber.setVelocityCmd(-Climber.ClimbCalibrateVel))
                    .onFalse(climber.setVelocityCmd(0.0));
            //Leaving cal, set any cal
            Cal.onFalse(Commands.runOnce(() -> {
                climber.setPosition(Climber.PowerUpPosition);
            }));

            // climber arm extend to max
            operator.povUp().onTrue(climber.armsToPoint(Climber.ExtendPosition));

            // climber arm to whatever drive team sets
            operator.povDown().onTrue(climber.armsToPoint(Climber.ClimbPositon));
            operator.x().onTrue(climber.armsSetpointCmd(Climber.PowerUpPosition));

            // manual flywheel speed adjustment
            operator.povLeft().onTrue(targeter.manualLow());
            operator.povRight().onTrue(targeter.manualHigh());

        } else {
            DriverStation.reportWarning("Comp Bindings: No operator bindings set, check controllers.", false);
        }

    }
}
