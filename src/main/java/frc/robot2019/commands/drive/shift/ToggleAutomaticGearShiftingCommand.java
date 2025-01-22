package frc.robot2019.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.GearShifterSubsystem;

public class ToggleAutomaticGearShiftingCommand extends InstantCommand {
    private GearShifterSubsystem gearShifter;

    public ToggleAutomaticGearShiftingCommand() {
        gearShifter = RobotContainer.getSubsystem(GearShifterSubsystem.class);
        addRequirements(gearShifter);
    }

    @Override
    public void execute() {
        if(gearShifter.getDefaultCommand().getName().equals("AutomaticGearShiftCommand")) {
            gearShifter.setDefaultCommand(null);
            gearShifter.autoshiftEnabled(false);
        } else {
            gearShifter.setDefaultCommand(new AutomaticGearShiftCommand());
            gearShifter.autoshiftEnabled(true);
        }
    }
}