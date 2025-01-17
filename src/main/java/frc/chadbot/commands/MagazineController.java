package frc.chadbot.commands;

import edu.wpi.first.wpilibj2.command.Command;
/**
 * MagazineController
 * 
 * Simple interface for feeding or ejecting ammo for shooter.
 * 
 * Low level calls could be used directly or if button binding is desired,
 * use the getFeedCmd() and getEjectCmd() calls and hook to a button
 */

public interface MagazineController {
    //low level
    public void feederOn();
    public void feederOff();
    public void ejectOn();
    public void ejectOff();

    public boolean safeToSpinUp();
    public int getBallCount();   

    // high level cmds
    public Command getEjectCmd();
    public Command getFeedCmd();
}
