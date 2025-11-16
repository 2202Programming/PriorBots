package frc.robot2025.subsystems.demo;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;

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

    // TODO set KFF to get vel close at mid speed, then other two as needed.
    double HW_kFF = 1.0/250.0;   // guess based on KV TUNE ME. It should spin, but vel will be off until tuned
    double HW_kP = 0.0;
    double HW_kI = 0.0;

    PIDFController posPid = new PIDFController(0.0, 0.0, 0.0, 0.0);   //TODO tune, this pid is run on rio
    PIDFController velHWPid = new PIDFController(HW_kP, 0.0, 0.0, HW_kFF);   //TODO tune these too, this just hold values for hw

    double SERVO_GR = 1.0; // TODO set gearing for converions factor [face-turns/mtr-turns] = [] non-dim

    // This can work in either Position or Velocity mode
    double cmdPos; //local copy of last commanded pos
    double cmdVel; //local copy of last commanded vel
  
    public CycloidalDrive(final int CANID) {
        setName("CycloidalDrive_"+CANID);
        // set our control constants for pos and vel pids
        servo = new NeoServo(CANID, posPid, velHWPid, false);

        // setup servo
        servo  // units should be [deg] and [deg/s]
            .setConversionFactor(SERVO_GR)
            .setTolerance(1.0, .01)  // [deg], [deg/s]
            .setSmartCurrentLimit(30, 5)  // [amp], [amp]
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
     * For position mode, expose the atSetpoint()
     */
    public boolean atSetpoint() {
        return servo.atSetpoint();
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


    public Command cmdPositionWait(double cmd_pos) {
        return Commands.sequence(
            cmdPosition(cmd_pos), 
            Commands.waitUntil(this::atSetpoint),
            Commands.print(getName() + " is atSetpoint " + cmd_pos) )
            .withName(getName()+":cmdPositionWait=" + cmd_pos);  
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
        
    }

    // Add a watcher so we can see stuff on network tables
    public WatcherCmd getWatcherCmd() {
        return this.new CDWatcher();
    }

    //TODO - Add simulation model
    

    // watcher will put values on the network tables for viewing elastic
    class CDWatcher extends WatcherCmd{
        CDWatcher(){
            //use newer form
            addEntry("analog_pos", CycloidalDrive.this.servo::getPosition, 2);
            addEntry("analog_vel", CycloidalDrive.this.servo::getVelocity, 2);
            // start the servo's NeoWatcher which has most of our stuff
            servo.getWatcher();
        }
    }
}
