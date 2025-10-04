package frc.robot2025.testBindings;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.robot2025.commands.GroundIntake.PickupSequence;
import frc.robot2025.commands.GroundIntake.PlaceSequence;
import frc.robot2025.commands.GroundIntake.BtmArmVel;
import frc.robot2025.commands.GroundIntake.BtmArmGoToPos;
import frc.robot2025.commands.GroundIntake.SetZero;
import frc.robot2025.commands.GroundIntake.TopArmVel;
import frc.robot2025.commands.GroundIntake.TopHold;

public class GITest {
    public static void myBindings(HID_Subsystem dc) {

        // get an xbox controller for the operator, or null
        CommandXboxController opr = (dc.Operator() instanceof CommandXboxController)
                ? (CommandXboxController) dc.Operator()
                : null;

        // Initialize PathPlanner, if we have the needed SS.
        // velocity commands for calibration
        opr.rightBumper().whileTrue(new BtmArmVel(30.0));
        opr.leftBumper().whileTrue(new BtmArmVel(-30.0));
        opr.povRight().whileTrue(new TopArmVel(30.0));
        opr.povLeft().whileTrue(new TopArmVel(-30.0));

        opr.povUp().onTrue(new BtmArmGoToPos(0.0));
        opr.povDown().onTrue(new SetZero()); // should be bound to an actual button but we dont have room rn -er
        opr.rightTrigger().whileTrue(new TopHold(5.0));

        // real pickup and place sequences
        opr.x().whileTrue(new PickupSequence("a"));
        opr.y().whileTrue(new PlaceSequence("a", -15.0)); // speed is slower than expected??
        opr.a().whileTrue(new PickupSequence("c"));
        opr.b().whileTrue(new PlaceSequence("c", -10.0));

    }
}