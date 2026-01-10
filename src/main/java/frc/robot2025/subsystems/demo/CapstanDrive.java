package frc.robot2025.subsystems.demo;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;

/*
 * Demo Subsystem for the Capstan Drive that Mr. Dean Spears built
 * 
 * Features an analog position sensor and very little backlash for gears.
 * 
 */
public class CapstanDrive extends SubsystemBase {
    final NeoServo servo;
    // controller for the motor
    final SparkBase servo_ctrlr;
    final SparkAnalogSensor servo_analog; 

    // TODO set KFF to get vel close at mid speed
    double arb_ff = 0.01;          // [% power] point where mechanism just starts to move
    final double HW_kFF = .01;      //tuned at 10 [deg/s] 
    final double HW_kP = 0.00025;
    final double HW_kI =0.00001;
    final double HW_kD = 0.0;
    final double HW_IZone = 2.50;

    //Pos Pid
    final double pos_kP = 2.0;
    final double pos_kI = 0.0;
    final double pos_kD = 0.0;
    final double pos_IZone = 10.0;  // allow some Ki when with in 10 degs

    // PIDS  HW is on the sparkmax controls vel, posPid is on RIO controls position [deg]
    PIDController posPid = new PIDController(pos_kP, pos_kI, pos_kD);   //TODO tune, this pid is run on rio
    PIDFController velHWPid = new PIDFController(HW_kP, HW_kI, HW_kD, HW_kFF);   //TODO tune these too, this just hold values for hw

    // Device physical values
    final double SERVO_GR = 4.0*4.0*5.0*8.65;  //inline and capstan [out rot/mtr rot] = []
    final double CONV_FACTOR = 360.0 / SERVO_GR ; // [deg]

    final boolean mtr_invert = true;    // so pos %pwr moves arm up
    final double MAX_POSITION =  45.0;  // [deg]
    final double MIN_POSITION = -45.0;  // [deg] 

    // Analog Sensor measured values for min/max position
    final double V0 = 1.740;      // [Volts] measured at zero position 
    final double Kv_p = 72.3327;  // [deg/volt]  120.0 / (Vmax - Vmin) measured
 
    // This actuator can work in either Position or Velocity mode
    double cmdPos; //local copy of last commanded pos
    double cmdVel; //local copy of last commanded vel

    // parameters
    
    double maxVel = 60.0;   // [deg/s]
    double maxAccel = 60.0; // [deg/s^2]

    public CapstanDrive(final int CANID) {
        setName("CapstanDrive_" + CANID);
        // set our control constants for pos and vel pids
        velHWPid.setIZone(HW_IZone);
        posPid.setIZone(pos_IZone);
        posPid.setIntegratorRange(-15.0, 15.0);

        servo = new NeoServo(CANID, posPid, velHWPid, mtr_invert);
        servo.setName(this.getName()+"/NeoServo-" + CANID);

        // setup servo
        servo  // units should be [deg] and [deg/s]
            .setConversionFactor(CONV_FACTOR)
            .setTolerance(0.5, 3.0)  // [deg], [deg/s]
            .setSmartCurrentLimit(35, 5)  // [amp], [amp]
            .setVelocityHW_PID(maxVel, maxAccel)
            .setMaxVelocity(maxVel)
            .setAFFVelocityComp(true);  // avoids stiction same value applied both dir 
        
        servo.setClamp(MIN_POSITION, MAX_POSITION);

        // get refs to servo Spark stuff.
        servo_ctrlr = servo.getController(); 
        servo_analog = servo_ctrlr.getAnalog();

        // initialize servo encoder with our powerup analog position
        var init_pos = getPositionAnalog();
        servo.setPosition(init_pos);
        cmdPos = init_pos;
        cmdVel = 0.0;
        setArbFF(arb_ff);
    }

    double getArbFF() {
        return arb_ff;
    }

    void setArbFF(double aff){
        this.arb_ff = aff;
        servo.setArbFeedforward(this.arb_ff);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("arb_ff",  this::getArbFF, this::setArbFF );
        builder.addDoubleProperty("max_vel", servo::getMaxVel, servo::setMaxVelocity);
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

    //These position/velocity are based on the internal motor counts and conversion factor
    //to put into engineering units.
    public double getPosition(){
        return servo.getPosition();
    }

    public double getVelocity()
    {
        return servo.getVelocity();
    }

    public double getPositionAnalog() {
        // calc the pos in deg from the measured voltage
        return (servo_analog.getVoltage() - V0) * Kv_p;
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
        //velocity cmds while held it should spin
        xbox.povLeft().whileTrue(this.cmdVelocity(10.0))
                      .onFalse(this.cmdVelocity(0.0));

        xbox.povRight().whileTrue(this.cmdVelocity(-10.0))
                       .onFalse(this.cmdVelocity(0.0));
        
        // Cmd to known points
        xbox.povUp().onTrue(this.cmdPosition(30.0));
        xbox.povDown().onTrue(this.cmdPosition(-30.0));
        xbox.y().onTrue(this.cmdPosition(0.0));
    }

    // Add a watcher so we can see stuff on network tables
    public WatcherCmd getWatcherCmd() {
        return this.new CDWatcher();
    }

    //TODO - Add simulation model
    public void simulationPeriodic() {
        servo.simulationPeriodic();
    }

    // watcher will put values on the network tables for viewing elastic
    class CDWatcher extends WatcherCmd{
        CDWatcher(){
            //use newer form
            addEntry("analog_pos", CapstanDrive.this::getPositionAnalog, 2);
            addEntry("analog_volt", CapstanDrive.this.servo_analog::getVoltage, 3);
            //motor count based pos/vel
            addEntry("internal_pos", CapstanDrive.this.servo::getPosition, 2);
            addEntry("internal_vel", CapstanDrive.this.servo::getVelocity, 2);
            addEntry("at_setpoint", CapstanDrive.this::atSetpoint);

            // other info about servo's motor
            addEntry("mtr_appliedOutput", CapstanDrive.this.servo_ctrlr::getAppliedOutput, 2);
            addEntry("mtr_OutputAmps", CapstanDrive.this.servo_ctrlr::getOutputCurrent, 2);
            addEntry("mtr_Temperature", CapstanDrive.this.servo_ctrlr::getMotorTemperature, 2);

            // start the servo's NeoWatcher which has most of our stuff
            servo.getWatcher();
        }
    }
}
