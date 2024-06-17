package frc.lib2202.command;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.lib2202.builder.RobotContainer;

public class PDPMonitorCmd extends WatcherCmd {
  private PowerDistribution pdp;
  NetworkTableEntry nt_total_power;
  NetworkTableEntry nt_temperature;
  NetworkTableEntry nt_voltage;
  NetworkTableEntry nt_drive_I;
  NetworkTableEntry nt_angle_I;

  public PDPMonitorCmd() {
    pdp = RobotContainer.getObjectOrNull("PDP");
  }

  @Override
  public String getTableName() {
    return "PDP";
  }

  @Override
  public void ntcreate() {
    NetworkTable table = getTable();
    nt_total_power = table.getEntry("power");
    nt_temperature = table.getEntry("temperature");
    nt_voltage = table.getEntry("voltage");
    nt_drive_I = table.getEntry("drive_I");
    nt_angle_I = table.getEntry("angle_I");
  }

  @Override
  public void ntupdate() {
    if (pdp == null) return;
    nt_total_power.setDouble(pdp.getTotalPower());
    nt_temperature.setDouble(pdp.getTemperature());
    nt_voltage.setDouble(pdp.getVoltage());
    nt_drive_I.setDouble(pdp.getCurrent(0));
    nt_angle_I.setDouble(pdp.getCurrent(1));
  }
}
