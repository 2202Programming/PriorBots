package frc.chadbot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.chadbot.subsystems.Positioner_Subsystem;
import frc.lib2202.builder.RobotContainer;

public class MovePositioner extends InstantCommand {
    PositionerMode mode;
    Positioner_Subsystem positioner;

    public enum PositionerMode {
        High, Low, Toggle
    }

    /**
     * Constructor
     * use this version to set a specific state
     * 
     * @param mode High --> shooter angle is high
     *             Low --> shooter angle is low
     *             Toggle --> change shooter angle to opposite
     */
    public MovePositioner(PositionerMode mode) {
        this.positioner = RobotContainer.getSubsystem(Positioner_Subsystem.class);
        addRequirements(positioner);
        // go where we are told
        this.mode = mode;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (mode == PositionerMode.High) {
            positioner.deploy();
        }
        if (mode == PositionerMode.Low) {
            positioner.retract();
        }
        if (mode == PositionerMode.Toggle) {
            if (positioner.isDeployed() == false) {
                positioner.deploy();
            } else {
                positioner.retract();
            }
        }

    }

}
