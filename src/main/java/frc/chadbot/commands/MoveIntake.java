package frc.chadbot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.chadbot.subsystems.Intake_Subsystem;
import frc.lib2202.builder.RobotContainer;

public class MoveIntake extends InstantCommand {
    Intake_Subsystem intake;
    DeployMode mode;

    public enum DeployMode {
        Deploy, Retract, Toggle
    }

    /**
     * Constructor
     * 
     * @param Mode Deploy --> intake will be deployed
     *             Retract --> intake will be retracted
     *             Toggle --> change intake mode
     */
    public MoveIntake(DeployMode mode) {
        this.intake = RobotContainer.getSubsystem(Intake_Subsystem.class);
        addRequirements(intake);
        // go where we are told
        this.mode = mode;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (mode == DeployMode.Deploy) {
            intake.deploy();
        }
        if (mode == DeployMode.Retract) {
            intake.retract();
        }
        if (mode == DeployMode.Toggle) {
            if (intake.isDeployed() == false) {
                intake.deploy();
            } else {
                intake.retract();
            }
        }
    }
}
