package frc.robot2025.subsystems.demo;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;
import frc.robot2025.commands.SpinCyclodialDrive;
/*
 * Demo Subsystem for the Cycloidal Drive that Mr. Dean Spears built
 * 
 * Cycloidal gear box around a dial face to show position.
 * 
 *  Note: I think this has an analog feedback sensor on the hw to measure position. - Mr.L.
 */
public class CycloidalDrive extends SubsystemBase {
    final NeoServo servo;
    // controller for the motor
    final SparkBase servo_ctrlr;
    final SparkAnalogSensor servo_analog;   // positon sensor, analog I think - Mr.L

    final double maxVel = 100.0;  // [deg/s]
    final double maxAccel = 75.0; // [deg/s^2]

    //NeoServo uses Softward PID for outer position loop and hw pid on controller for Velocity
    final PIDFController posPid;    // outer positon pid - runs on Rio
    final PIDFController velHWPid;  // holds pids values, pid is run in vel-mode on hw controller 

    double SERVO_GR = 1.0; // TODO set gearing for converions factor [face-turns/mtr-turns] = [] non-dim

    // TODO set KFF to get vel close at mid speed, then other two as needed.
    double HW_kFF = 1.0/550.0;   // guess based on KV TUNE ME. It should spin, but vel will be off until tuned
    double HW_kP = 0.0;
    double HW_kI = 0.0;

    // This can work in either Position or Velocity mode
    double cmdPos; //local copy of last commanded pos
    double cmdVel; //local copy of last commanded vel
  
    public CycloidalDrive(final int CANID) {
        setName("CycloidalDrive_"+CANID);
        // set our control constants for pos and vel pids
        posPid = new PIDFController(1.0, 0.0, 0.0, 0.0);   //TODO tune, this pid is run on rio
        velHWPid = new PIDFController(HW_kP, 0.0, 0.0, HW_kFF);  //TODO tune these too, this just hold values for hw
        servo = new NeoServo(CANID, posPid, velHWPid, false);

        // setup servo
        servo  // units should be [deg] and [deg/s]
            .setConversionFactor(SERVO_GR)
            .setTolerance(1.0, .01)  // [deg], [deg/s]
            .setSmartCurrentLimit(10, 1)  // [amp], [amp]
            .setVelocityHW_PID(maxVel, maxAccel)
            .setMaxVelocity(maxVel);
        
        // get refs to servo Spark stuff.
        servo_ctrlr = servo.getController();
        servo_analog = servo_ctrlr.getAnalog();        
        init();
    }

    protected void init(){
        // read hardware's starting position
        double cur_pos = servo_analog.getPosition();  // TODO - this may need debugging HW is new...
        // initialize position based on feedback 
        servo.setPosition(cur_pos);  // doesn't move, just tells it where it is.
    }

    @Override
    public void periodic() {
        servo.periodic();
    }

    /*
     * Switches to position mode
     */
    public void setSetpoint(double pos) {
        cmdPos = pos;  //local copy
        servo.setSetpoint(cmdPos);
    }

    /*
     * Switches to velocity mode
     */
    public void setVelocity(double vel) {
        cmdVel = vel;   //local copy
        servo.setVelocityCmd(vel);
    }

    public double getVelocity()
    {
        return servo.getVelocity();
    }

    // Add simple commands here - alternative to puttiing Commands in their own file.
    // This pattern is new for us, but is simpler. 
    public Command cmdVelocity(double cmd_vel){
        return runOnce(() -> {
            this.setVelocity(cmd_vel);  //switches Neo to vel mode
        });
    }

    public Command cmdPosition(double cmd_pos){
        return runOnce(() -> {
            this.setSetpoint(cmd_pos);  //switches Neo to position mode
        });
    }


    // Add a DEMO bindings - we don't normally do this but for a demo 
    // it is handy because bot-on-board spec files change frequently
    // as they are used during the season.
    public void setDemoBindings(CommandXboxController xbox) {
        //bindings for Cycloid demo - use POV buttons with new ss cmd pattern
        xbox.povLeft().onTrue(this.cmdVelocity(10.0));
        xbox.povRight().onTrue(this.cmdVelocity(-10.0));
        xbox.povUp().onTrue(this.cmdPosition(0.0));
        xbox.povDown().onTrue(this.cmdPosition(180.0));
        
        // test @Tylers spin command
        xbox.rightBumper().onTrue(new SpinCyclodialDrive(20.0));
        xbox.leftBumper().onTrue(new SpinCyclodialDrive(-20.0));
        xbox.rightTrigger(0.5).onTrue(this.cmdVelocity(0.0)); //stop in vel-mode
    }

    // Add a watcher so we can see stuff on network tables
    public WatcherCmd getWatcherCmd() {
        return this.new CDWatcher();
    }

    //TODO - Add simulation model
    

    // watcher will put values on the network tables for viewing elastic
    class CDWatcher extends WatcherCmd{
        // start the servo's NeoWatcher which has most of our stuff
        Command NeoWatcher = servo.getWatcher();
        
        //aditional entries 
        NetworkTableEntry nt_analog_pos;
        NetworkTableEntry nt_analog_vel;

        @Override
        public String getTableName() {
            return CycloidalDrive.this.getName();
        };

        @Override
        public void ntcreate() {
             NetworkTable table = getTable();
           nt_analog_pos = table.getEntry("analog_pos");
           nt_analog_vel = table.getEntry("analog_vel");
        }

        @Override
        public void ntupdate() {     
            nt_analog_pos.setDouble(fmt2(servo_analog.getPosition()));
            nt_analog_vel.setDouble(fmt2(servo_analog.getVelocity()));
        }        
    }  //watcher
}
