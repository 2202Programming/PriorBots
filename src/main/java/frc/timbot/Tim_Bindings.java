package frc.timbot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.timbot.commands.FrisbeeShoot;
import frc.timbot.subsystem.ShooterLifter;
import frc.timbot.subsystem.Feeder;
import frc.timbot.subsystem.Shooter.Shooter;

public final class Tim_Bindings {

    public static void setBindings(){
        HID_Subsystem dc = RobotContainer.getSubsystem("DC");
        Shooter shooter = RobotContainer.getSubsystem("shooter");
        Feeder feeder = RobotContainer.getSubsystem(Feeder.class);
        ShooterLifter sl = RobotContainer.getSubsystem(ShooterLifter.class);
        //LifterMove upPos;
        var driver = dc.Driver();
        if (driver instanceof  CommandXboxController) {
            CommandXboxController xbox_driver = (CommandXboxController)driver;
            //shooter.setTestBindings(xbox_driver);
            xbox_driver.rightBumper().whileTrue(shooter.cmdVelocity(20));
            xbox_driver.rightBumper().onTrue(new PrintCommand("right bumper has been pressed"));
            xbox_driver.rightBumper().onFalse(shooter.cmdVelocity(0));

            // xbox_driver.leftBumper().whileTrue(shooter.cmdVelocityBack(10));
            // xbox_driver.leftBumper().onTrue(new PrintCommand("left bumper has been pressed"));
            // xbox_driver.leftBumper().onFalse(shooter.cmdVelocityBack(0));

            // Manual clear for Feeder
            xbox_driver.leftTrigger().onTrue(feeder.feeder_fire());
            xbox_driver.leftTrigger().onFalse(feeder.feeder_reset());
            // Command for shooting when flywheel is at speed
            //xbox_driver.a().onTrue(new ConditionalCommand(new FrisbeeShoot(), new WaitCommand(1.0), shooter::isAtShootSpeed));
            /*Command for spinning up, wait until it is spun up
            Repeat: shoot, and then check if it is spun up again,
            At the end: make the shooter reset, and stop the motor
            */ 
            xbox_driver.rightTrigger().whileTrue( 
                new SequentialCommandGroup(
                    shooter.cmdVelocityWait(53.12),
                    new FrisbeeShoot() 
                    ).repeatedly());
            xbox_driver.rightTrigger().onFalse(
                new ParallelCommandGroup(
                    feeder.feeder_reset(),
                    shooter.cmdVelocity(0)
                ));

            xbox_driver.povUp().onTrue(sl.cmdHeight(14));
            xbox_driver.povDown().onTrue(sl.cmdHeight(5));
            xbox_driver.povLeft().onTrue(sl.cmdHeight(0.5));

        //SmartDashboard.putNumber("Position", upPos.get_height());
        }
        else {
            DriverStation.reportError("Timbot expects xbox controller, no driver bindings set, check controllers.", false);
        }
    }
}