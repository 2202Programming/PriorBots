package frc.robot2026.testBindings;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.TargetCentricDrive;
import frc.robot2026.command.shooter.AutoShoot;
import frc.robot2026.subsystems.Hopper;
import frc.robot2026.subsystems.Shooter.Targeter;

public class JRtestBinding {

    // used to check rotation rates when calibrating wheel speeds
    public static void calbrate(CommandXboxController c) {
        Targeter targeter = RobotContainer.getSubsystemOrNull(Targeter.class);
        Hopper hopper = RobotContainer.getSubsystem(Hopper.class);
        c.rightBumper().onTrue(new TargetCentricDrive(targeter).withName("TargeterCmd"));
        
        // Shoot with targetSpeed based on distance to hub
        c.leftTrigger(0.7).whileTrue(new AutoShoot("left", targeter::getTargetSpeed, targeter::getTolerance, 1.0));
        c.leftTrigger(0.7)
                .whileTrue(new AutoShoot("right", targeter::getTargetSpeed, targeter::getTolerance, 1.0));
        c.leftTrigger(0.1).whileTrue(hopper.cmdBeltPct(0.705))
                .onFalse(hopper.cmdBeltPct(0));
    }
}
