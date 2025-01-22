package frc.robot2019.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.DriveTrainSubsystem;

public class InvertDriveControlsCommand extends InstantCommand {
   final private DriveTrainSubsystem driveTrain;

    public InvertDriveControlsCommand() {
        driveTrain = RobotContainer.getSubsystem(DriveTrainSubsystem.class);
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.invertControls();
    }
}