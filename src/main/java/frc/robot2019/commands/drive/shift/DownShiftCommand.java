package frc.robot2019.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.GearShifterSubsystem;

public class DownShiftCommand extends InstantCommand {
    final GearShifterSubsystem gearShifter;
    public DownShiftCommand() {
        gearShifter = RobotContainer.getSubsystem(GearShifterSubsystem.class);
        addRequirements(gearShifter);
    }

    @Override
    public void execute() {
        gearShifter.shiftDown();
    }
}