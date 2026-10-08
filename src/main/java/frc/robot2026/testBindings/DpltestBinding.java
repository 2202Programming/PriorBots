package frc.robot2026.testBindings;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import frc.lib2202.command.swerve.FaceToTag;
import frc.lib2202.command.swerve.RotateTo;
import frc.lib2202.command.swerve.calibrate.TestConstantVelocity;
import frc.lib2202.command.swerve.calibrate.TestRotateVelocity;
import frc.robot2026.BindingsCompetition;
import frc.robot2026.RegisteredCommands;
import frc.robot2026.command.autoClimberCommand;

public class DpltestBinding {

    //used to check rotation rates when calibrating wheel speeds
    public static void calbrate(CommandXboxController c) {
        c.rightBumper().onTrue(new TestRotateVelocity (90.0,4.0)); //360 deg
        c.leftBumper().onTrue(new TestRotateVelocity (30.0,6.0));  //180 deg
        c.leftTrigger().onTrue(new TestConstantVelocity(0.25, 4.0)); //moves 1m
        c.rightTrigger().onTrue(new TestConstantVelocity(0.25, 8.0)); //moves 2m

        // Test named command Shoot off a button...
        c.a().whileTrue(RegisteredCommands.ncShoot());
        c.b().onTrue(
            //new FaceToTag(10, 5.0)
            new RotateTo(BindingsCompetition.targeter.getRedHub(),
                         BindingsCompetition.targeter.getBlueHub(),3.0).setP(4.0));
        c.x().onTrue(new autoClimberCommand(false) );  // good for testing with Pos3 starts
    }
}
