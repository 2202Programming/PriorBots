package frc.robot2026.testBindings;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot2026.command.autoClimberCommand;
//import frc.robot2026.command.shooter.AutoShoot;

public class GRLTestBindings {

    @SuppressWarnings("unused")
    public static void calbrate(CommandXboxController c) {
        c.a().onTrue(new autoClimberCommand(false));
        c.b().onTrue(new autoClimberCommand(true));

    }
}

