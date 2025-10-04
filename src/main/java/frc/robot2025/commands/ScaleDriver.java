package frc.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.hid.HID_Subsystem;

public class ScaleDriver extends Command {
    
    final HID_Subsystem dc;
    final double xy_factor;
    final double rot_factor;

    public ScaleDriver(double factor) {
        this(factor, factor);
    }

    public ScaleDriver(double xy_factor, double rot_factor) {
        this.xy_factor = xy_factor;
        this.rot_factor = rot_factor;
        dc = RobotContainer.getObject("DC");
    }

    @Override
    public void initialize() {
        if (dc == null) return;
        dc.setStickScale(xy_factor, rot_factor);
    }
    
    @Override
    public void end(boolean interrupted) {
        //return to normal
        dc.setStickScale(1.0, 1.0);
    }

}
