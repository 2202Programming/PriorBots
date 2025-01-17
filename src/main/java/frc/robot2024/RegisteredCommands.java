package frc.robot2024;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.FaceToTag;
import frc.lib2202.command.swerve.RotateTo;
import frc.lib2202.command.swerve.RotateUntilSeeTags;
import frc.robot2024.Constants.Tag_Pose;
import frc.robot2024.commands.Intake.EjectNote;
import frc.robot2024.commands.Intake.IntakeSequence;
import frc.robot2024.commands.Shooter.ShooterSequence;
import frc.robot2024.commands.Shooter.ShooterServoSequence;
import frc.robot2024.subsystems.Intake;

/*
 * Place commands named in PathPlaner autos here.
 */
public class RegisteredCommands {

    //Timeouts allow paths to continue in auto even if we miss a Note.   
    static final double ShooterTimeOut = 3.0;

    public static SendableChooser<Command> RegisterCommands() {
        SendableChooser<Command> autoChooser;
        var intake = RobotContainer.getSubsystem(Intake.class);

        // NamedCommands for use in PathPlanner scripts.
        NamedCommands.registerCommand("intakeDown", 
             new InstantCommand( () -> {
                intake.setAngleSetpoint(Intake.DownPos);
                intake.setIntakeSpeed(Intake.RollerMaxSpeed);
            }));

        NamedCommands.registerCommand("pickup", 
            new IntakeSequence(true));
        
        NamedCommands.registerCommand("eject", 
            new EjectNote());
        
        if (RobotContainer.getRobotName().equals("CompetitionBotAlpha2024")) {// Just for alpha
            NamedCommands.registerCommand("shoot",
                new ShooterSequence(true, 3500.0).withTimeout(ShooterTimeOut));
            
            NamedCommands.registerCommand("angle_shoot",
                new SequentialCommandGroup(
                    new RotateTo(Tag_Pose.ID4, Tag_Pose.ID7), 
                    new ShooterSequence(3200.0)).withTimeout(ShooterTimeOut));
        } else {
            NamedCommands.registerCommand("shoot",  
                    new ShooterServoSequence(true).withTimeout(ShooterTimeOut));
            
            NamedCommands.registerCommand("angle_shoot",
                new SequentialCommandGroup(
                    new RotateUntilSeeTags(Tag_Pose.ID4, Tag_Pose.ID7), 
                    new FaceToTag(100),//HACK Does not matter
                    new ShooterServoSequence(true).withTimeout(ShooterTimeOut)));
        
            NamedCommands.registerCommand("RotateTo", 
                new RotateUntilSeeTags(Tag_Pose.ID4, Tag_Pose.ID7));
            NamedCommands.registerCommand("highShoot", new ShooterServoSequence(45.5,3000,false,true));
            NamedCommands.registerCommand("highShoot2500", new ShooterServoSequence(45.5,2500,false,true));
            NamedCommands.registerCommand("midShoot", new ShooterServoSequence(39,3000.0,false,true));
        }
        
        // check autobuilder is setup
        autoChooser = (AutoBuilder.isConfigured()) ? AutoBuilder.buildAutoChooser() : null;
        // select our auto
        SmartDashboard.putData("Auto Chooser", autoChooser);
        return autoChooser;
    }
}