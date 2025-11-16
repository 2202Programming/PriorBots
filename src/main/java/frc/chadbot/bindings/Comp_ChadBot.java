package frc.chadbot.bindings;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.chadbot.Constants.Shooter;
import frc.chadbot.commands.IntakeCommand;
import frc.chadbot.commands.IntakeCommand.IntakeMode;
import frc.chadbot.commands.MagazineGatedCommand;
import frc.chadbot.commands.MoveIntake;
import frc.chadbot.commands.MoveIntake.DeployMode;
import frc.chadbot.commands.MovePositioner;
import frc.chadbot.commands.MovePositioner.PositionerMode;
import frc.chadbot.commands.Shoot.VelShootCommand;
import frc.chadbot.subsystems.Magazine_Subsystem;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.pathing.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.command.swerve.TargetCentricDrive;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.robot2024.Constants.Tag_Pose;

/*
 * Please don't edit this without leads/mentor/driveteam review
 */

public final class Comp_ChadBot {

    
    

    public static void ConfigureCompetition(HID_Subsystem dc) {
        DriverBinding(dc);
        OperatorBindings(dc);
    }


    private static void DriverBinding(HID_Subsystem dc) {
        CommandXboxController driver;

        if (dc.Driver() instanceof CommandXboxController) {
            driver = (CommandXboxController)dc.Driver();
        } else {
            DriverStation.reportError("BindingsCompetition: Please use XBOX controller for Driver", false);
            return;
        }

        var drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);

        // Driver buttons
        driver.leftTrigger().whileTrue(new RobotCentricDrive(drivetrain, dc));
        driver.y().onTrue(new AllianceAwareGyroReset());
        driver.rightTrigger().whileTrue(new TargetCentricDrive(Tag_Pose.ID4, Tag_Pose.ID7));
        
        /*driver.b().whileTrue(new IntakeCommand(IntakeMode.ExpellCargo));

        driver.x().whileTrue(new IntakeCommand(IntakeMode.LoadCargo));*/    }

    static void OperatorBindings(HID_Subsystem dc) {
        MagazineGatedCommand mag_default_cmd;
        mag_default_cmd = new MagazineGatedCommand(1.0);
        // set default command for mag, normally done in RobotSpec file
        Magazine_Subsystem mag = RobotContainer.getSubsystem(Magazine_Subsystem.class);
        mag.setDefaultCommand(mag_default_cmd);
        
        @SuppressWarnings("unused")
        var sideboard = dc.SwitchBoard();

        CommandXboxController operator;

        if (dc.Operator() instanceof CommandXboxController) {
            operator = (CommandXboxController)dc.Operator();
        } else {
            DriverStation.reportError("BindingsCompetition: Please use XBOX controller for Operator", false);
            return;
        }

        // shoot commands
        operator.povLeft().whileTrue(new VelShootCommand(Shooter.longVelocity));


        //intake/expel commands
        operator.leftBumper().onTrue(new MoveIntake(DeployMode.Toggle));
        operator.a().whileTrue(new IntakeCommand((() -> 0.6), () -> 0.5, IntakeMode.LoadCargo));
        operator.b().whileTrue(new IntakeCommand((() -> 0.35), () -> 0.5, IntakeMode.ExpellCargo));

        // positioner binds
        operator.rightBumper().onTrue(new MovePositioner(PositionerMode.Toggle));

         operator.x().whileTrue(mag_default_cmd.getFeedCmd());
         operator.y().whileTrue(mag_default_cmd.getEjectCmd());



       
    }
}
    
