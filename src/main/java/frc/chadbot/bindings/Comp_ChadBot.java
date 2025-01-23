package frc.chadbot.bindings;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chadbot.commands.IntakeCommand;
import frc.chadbot.commands.IntakeCommand.IntakeMode;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.command.swerve.TargetCentricDrive;
import frc.lib2202.subsystem.hid.HID_Xbox_Subsystem;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.robot2024.Constants.Tag_Pose;
import frc.robot2024.commands.Climber.Climb;
import frc.robot2024.commands.Intake.AngleCalibration;
import frc.robot2024.commands.Intake.EjectNote;
import frc.robot2024.commands.Intake.InIntake;
import frc.robot2024.commands.Intake.IntakeSequence;
import frc.robot2024.commands.Intake.MoveToAnglePos;
import frc.robot2024.commands.Intake.TestIntake;
import frc.robot2024.commands.Shooter.CalibrateWithLS;
import frc.robot2024.commands.Shooter.ShooterAngleSetPos;
import frc.robot2024.commands.Shooter.ShooterAngleVelMove;
import frc.robot2024.commands.Shooter.ShooterServoSequence;
import frc.robot2024.commands.Shooter.ShooterServoSequenceDebug;
import frc.robot2024.commands.auto.AutoShooting;
import frc.robot2024.commands.auto.AutoShooting.ShootingTarget;
import frc.robot2024.subsystems.AmpMechanism;
import frc.robot2024.subsystems.Climber;
import frc.robot2024.subsystems.Intake;

/*
 * Please don't edit this without leads/mentor/driveteam review
 */
public final class Comp_ChadBot {

    public static void ConfigureCompetition(HID_Xbox_Subsystem dc) {
        DriverBinding(dc);
        OperatorBindings(dc);
    }


    private static void DriverBinding(HID_Xbox_Subsystem dc) {
        CommandXboxController  driver;
        if (dc.Driver() instanceof CommandXboxController) {
            driver = (CommandXboxController)dc.Driver();
        } else {
            DriverStation.reportError("BindingsCompetition: Please use XBOX controller for Driver", false);
            return;
        }

        var drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);

        // Driver buttons
        driver.leftTrigger().whileTrue(new RobotCentricDrive(drivetrain, dc));
        driver.y().onTrue(new AllianceAwareGyroReset(true));
        driver.rightTrigger().whileTrue(new TargetCentricDrive(Tag_Pose.ID4, Tag_Pose.ID7));
        
        driver.b().onTrue(new IntakeCommand(IntakeMode.ExpellCargo));
        driver.b().onFalse(new IntakeCommand(IntakeMode.Stop));

        driver.x().onTrue(new IntakeCommand(IntakeMode.LoadCargo));
        driver.x().onFalse(new IntakeCommand(IntakeMode.Stop));
    }


    static void OperatorBindings(HID_Xbox_Subsystem dc) {
        var sideboard = dc.SwitchBoard();
        CommandXboxController operator;
        if (dc.Operator() instanceof CommandXboxController) {
            operator = (CommandXboxController)dc.Operator();
        }else {
            DriverStation.reportError("BindingsCompetition: Please use XBOX controller for Operator", false);
            return;
        }




        //Trigger ManualShoot = sideboard.sw16();
        //Trigger ShooterCalibrate = sideboard.sw12();
        //Trigger IntakeCalibrate = sideboard.sw13();

        // Switchboard buttons too
        //sideboard.sw23().onTrue(new MoveToAnglePos(Intake.DownPos, Intake.TravelUp));

        /***************************************************************************************/
        // REAL COMPETITION BINDINGS.
        
        //operator.b().whileTrue(new EjectNote()); // eject note from intake
        //operator.x().whileTrue(new InIntake(false)); // works ---> seq for stay in intake for amp shoot
        //IntakeCalibrate.and(operator.povUp()).onTrue(new AngleCalibration(-25.0));// intake calibrate
        //IntakeCalibrate.and(operator.povDown()).whileTrue(new TestIntake(0.0));
           
        // speaker shooting                                                                                            
       // ManualShoot.and(operator.rightTrigger()).onTrue(new ShooterServoSequence()); // was 35
        //ManualShoot.and(operator.leftTrigger()).onTrue(new ShooterServoSequenceDebug());

        // AutoShootm 
        //ManualShoot.negate().and(operator.rightBumper())
           // .onTrue(new AutoShooting(ShootingTarget.Speaker, 45.0, 3000.0));
        //ManualShoot.negate().and(operator.rightTrigger())
           // .onTrue(new AutoShooting(ShootingTarget.Speaker, 36.0, 3200.0));
        
        // Calibration commands
        //ShooterCalibrate.and(operator.povUp()).onTrue(new CalibrateWithLS()); 
        //ShooterCalibrate.and(operator.povDown()).whileTrue(new ShooterAngleVelMove(-2.0));

    }
}
    
