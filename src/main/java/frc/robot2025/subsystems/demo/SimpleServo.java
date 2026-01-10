package frc.robot2025.subsystems.demo;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.command.WatcherCmd;

public class SimpleServo extends SubsystemBase {

    //Futaba s-28 0-180 degee settings (full range)
    final int MIN= 360;            //[uS] for 0.0 setpoint
    final int DEADBAND_MIN = 1278;
    final int CENTER = 1280;       //[uS] for 0.5 setpoint
    final int DEADBAND_MAX = 1282;
    final int MAX= 2200;           //[uS] for 1.0 setpoint
    final double RANGE = 180.0;    //[deg] full range of travel
    final double time_full = RANGE/(60.0/.24); //[sec] servo travel time for full range

    final PWM servo;
    double pos_tollerence = 0.02;  //2%
    double setpoint;               //[] range 0.0 - 1.0
    // model for estimated position
    LinearFilter model;
    double est_pos;      //position based on the servo's model

    public SimpleServo(int PWMID){
        servo = new PWM(PWMID);
        servo.setBoundsMicroseconds(MAX, DEADBAND_MAX, CENTER, DEADBAND_MIN, MIN);
        setName("SimpleServo_"+PWMID);

        //use a first order IIR filter with atc = full_travel time * 63.2% rule (1 tc gets to 63.2% of travel)
        //short tc, faster response. larger tc, slower response
        double tc = time_full * 0.632;
        model = LinearFilter.singlePoleIIR(tc, 0.02);
        model.reset();
        setSetpoint(0);
    }
 
    public void periodic(){
        //run our model with setpoint (commande pos)
        est_pos = model.calculate(setpoint);
    }

    public void setSetpoint(double pos_cmd){
        this.setpoint = pos_cmd;
        servo.setPosition(this.setpoint);
    }

    public double getPosition(){ 
        //servo.getPosition() just tracks the setpoint with 0 delay, this models the speed
        return est_pos;       
    }

    // this is here for testing float base entry in watcher
    public float getPositionAsFloat(){ 
        //servo.getPosition() just tracks the setpoint with 0 delay, this models the speed
        return (float)est_pos;       
    }

    public boolean atSetpoint() {
        return Math.abs(setpoint - getPosition()) < pos_tollerence;
    }

    // Command Interface
    public Command cmdPosition(double pos) {
        // shorthand for an instant command using lambda 
        return runOnce(() -> {           
             this.setSetpoint(pos);
        });
    }

    public Command cmdPositionWaitForModel(double pos) {
        return Commands.sequence(
            cmdPosition(pos), 
            Commands.waitUntil(this::atSetpoint),
            Commands.print(getName() + " is done moving to " + pos + ", indicated by model.") )
            .withName(getName()+":cmdPosWaitforModel="+pos);  //best-practice use a good name
    }

    public WatcherCmd getWatcherCmd() {
        return this.new SimpleServoWatcher();        
    }

    class SimpleServoWatcher extends WatcherCmd {                    
        SimpleServoWatcher() {            
            addEntry("position", SimpleServo.this::getPosition, 3);
            addEntry("position_fl", SimpleServo.this::getPositionAsFloat, 2);
            addEntry("setpoint", ()-> {return setpoint;});
            addEntry("atSetpoint", SimpleServo.this::atSetpoint);            
        }       
    }
}
