package frc.robot2026.testBindings;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
import frc.robot2026.command.intake.Burp;
import frc.robot2026.subsystems.Intake;

public class BGTestBindings {

    @SuppressWarnings("unused")
    public static void calbrate(CommandXboxController c) {

        // final Shooter shooterL = RobotContainer.getSubsystem("shooter_left");
        // final Shooter shooterR = RobotContainer.getSubsystem("shooter_right");

        // final Indexer indexerL = RobotContainer.getSubsystem("lIndexer");
        // final Indexer indexerR = RobotContainer.getSubsystem("rIndexer");

        final Intake intake = RobotContainer.getSubsystem("intake");

        // Hopper h = RobotContainer.getSubsystemOrNull(Hopper.class);

       c.a().onTrue(intake.cmdSetPos(1.0));
       c.b().onTrue(intake.cmdZeroPos()); 

       c.x().onTrue(new Burp(-0.5, 20.0));
        
    }
}
