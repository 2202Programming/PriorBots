package frc.lib2202.command;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

/*
 * TargetWatcherCmd - puts angle and RPM into network talbe
 *  and provides standard interface for shoot commands.
 * 
 */
public abstract class TargetWatcherCmd extends WatcherCmd {
    
    final String AngleName = "Angle";
    final String RPMName = "RPM";
    final String DistanceName = "Distance";

    NetworkTableEntry nt_angle;
    NetworkTableEntry nt_rpm;
    NetworkTableEntry nt_distance;

    @Override
    public void ntcreate() {
        NetworkTable table = getTable();
        nt_angle = table.getEntry(AngleName);
        nt_rpm = table.getEntry(RPMName);
        nt_distance = table.getEntry(DistanceName);
    }

    @Override
    public void ntupdate() {
        nt_angle.setDouble( getTargetAngle());
        nt_rpm.setDouble(getTargetRPM());
        nt_distance.setDouble(getTargetDistance());
    }

    @Override
    public void execute(){      
        calculate();
        super.execute(); //nt work done
    }

    public abstract void calculate();
    public abstract double getTargetAngle();
    public abstract double getTargetRPM();
    public abstract double getTargetDistance();
}
