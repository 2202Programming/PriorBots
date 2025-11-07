package frc.robot2025.subsystems.demo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.command.WatcherCmd;

public class SimpleServo extends SubsystemBase {
    
    //Futaba s-28 0-180 degee settings (full range)
    final int MIN= 360;            //[uS] for 0.0 setpoint
    final int DEADBAND_MIN = 1278;
    final int CENTER = 1280;       // [uS] or 0.5 setpoint
    final int DEADBAND_MAX = 1282;
    final int MAX= 2200;           // [uS] for 1.0 setpoint

    final PWM servo;
    double setpoint;

    public SimpleServo(int PWMID){
        servo = new PWM(PWMID);
        servo.setBoundsMicroseconds(MAX, DEADBAND_MAX, CENTER, DEADBAND_MIN, MIN);
        setName("SimpleServo_"+PWMID);
        //var foo = servo.getBoundsMicroseconds();
    }
 
    public void setSetpoint(double pos_cmd){
        this.setpoint = pos_cmd;
        servo.setPosition(this.setpoint);
    }

    public double getPosition(){ 
        return servo.getPosition();
    }

    public Command cmdPosition(double pos) {
        // shorthand for an instant command using lambda 
        return runOnce(() -> {            
             this.setSetpoint(pos);
        });
    }

    public WatcherCmd getWatcherCmd() {
        return this.new SimpleServoWatcher();        
    }

    class SimpleServoWatcher extends WatcherCmd{
        NetworkTableEntry nt_setpoint;
        NetworkTableEntry nt_position;
        
        @Override
        public String getTableName() {
            return SimpleServo.this.getName();
        };
        
        @Override
        public void ntcreate() {
            NetworkTable table = getTable();
            nt_setpoint = table.getEntry("setpoint");
            nt_position = table.getEntry("position");            
        }

        @Override
        public void ntupdate() {
            nt_position.setDouble(fmt2(getPosition()));
            nt_setpoint.setDouble(fmt2(setpoint));
        }        
    }
}
