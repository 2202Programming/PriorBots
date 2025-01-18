package frc.robot2019.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.GearShifterSubsystem;

public class UpShiftCommand extends InstantCommand {
    final GearShifterSubsystem shifter;
    public UpShiftCommand() {
        shifter = RobotContainer.getSubsystem(GearShifterSubsystem.class);
        addRequirements(shifter);
    }

    @Override
    public void execute() {
        shifter.shiftUp();
    }
}