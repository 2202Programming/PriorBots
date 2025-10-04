package frc.robot2025.testBindings;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.robot2025.commands.EndEffectorPercent;
import frc.robot2025.commands.WristToPos;

@Deprecated  // switched to WristFLA
public class EndEffectorTest {
    public static void myBindings(HID_Subsystem dc) {

        // get an xbox controller for the operator, or null
        CommandXboxController opr = (dc.Operator() instanceof CommandXboxController)
                ? (CommandXboxController) dc.Operator()
                : null;

        // try PS2
        if (opr == null && dc.Operator() instanceof CommandPS4Controller) {
            CommandPS4Controller opp = (CommandPS4Controller) dc.Driver();
            opp.square().onTrue(new WristToPos(1.0));
            opp.triangle().onTrue(new WristToPos(0.0));
            opp.cross().onTrue(new WristToPos(0.5));
            return;
        }

        // for end effector
        opr.rightBumper().whileTrue(new EndEffectorPercent(-.3, "rightBumper")); // reverse
        opr.rightTrigger().whileTrue(new EndEffectorPercent(.5, "rightTrigger")); // p

        // opr.x().whileTrue(new backupEE_Move(1000.0));
        opr.x().onTrue(new WristToPos(1.0, "x"));
        opr.y().onTrue(new WristToPos(0.0, "y"));
        opr.a().onTrue(new WristToPos(0.5, "a"));

    }

}
