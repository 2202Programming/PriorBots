package frc.robot2025.testBindings;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.robot2025.commands.SpinCyclodialDrive;

public class TylerCommands {

    public static void myBindings(HID_Subsystem dc) {
        // Elevator_Subsystem elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
        // WristFLA wrist = RobotContainer.getSubsystem(WristFLA.class);
        // get an xbox controller for the operator, or null
        CommandXboxController opr = (dc.Operator() instanceof CommandXboxController)
                ? (CommandXboxController) dc.Operator()
                : null;

        // bail if there was no opr controller of type Xbox
        if (opr == null) return;

        // Press X On Controller to Spin while button is held
        opr.x().whileTrue(new SpinCyclodialDrive(30.0,false))
               .onFalse(new SpinCyclodialDrive(0.0,false));

        // Press B On Controller to Spin opposite way
        opr.b().whileTrue( new SpinCyclodialDrive(-30.0,false))
               .onFalse(new SpinCyclodialDrive(0.0,false));
               
        // Press Y On Controller to Stop Spinning
        opr.y().onTrue(new SpinCyclodialDrive(0.0,false)
            .withName("SpinCycZeroVel"));

        // Press Down Arrow on Controller to go to 0
        opr.povDown().onTrue(new SpinCyclodialDrive(0,true));
    }
}