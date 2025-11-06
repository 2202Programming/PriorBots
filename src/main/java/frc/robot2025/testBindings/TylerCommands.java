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

        //PRESS X ON CONTROLLER
        opr.x().whileTrue(new SpinCyclodialDrive(30.0));
    }
}