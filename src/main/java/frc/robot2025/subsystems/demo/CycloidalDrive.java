package frc.robot2025.subsystems.demo;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;

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
    final double maxAccel = 75.0; // [deg/s^2]

    // TODO set KFF to get vel close at mid speed, then other two as needed.
    final double HW_kFF = .000291;  // tuned at 120 & 300 deg/s witn aff set
    final double HW_kP = 0.00006;
    final double HW_kI = 0.0000052;
    final double HW_kD = 0.0025;
    final double HW_IZone = 50;

    //Pos Pid
    final double pos_kP = 15.0;
    final double pos_pI = 0.0;
    final double pos_pD = 0.3;
    final double pos_IZone = 5.0;

    // PIDS  HW is on the sparkmax controls vel, posPid is on RIO controls position [deg]
    PIDController posPid = new PIDController(pos_kP, pos_pI, pos_pD);   //TODO tune, this pid is run on rio
    PIDFController velHWPid = new PIDFController(HW_kP, HW_kI, HW_kD, HW_kFF);   //TODO tune these too, this just hold values for hw
    final double SERVO_GR = 1.0 / 15.0; // [face-rotations/mtr-rotations] = []
    final double CONV_FACTOR = 360.0 * SERVO_GR ; // [deg]
 
    // This actuator can work in either Position or Velocity mode
    double cmdPos; //local copy of last commanded pos
    double cmdVel; //local copy of last commanded vel

    // parameters
    double arb_ff = 0.012;  // [% power] point where mechanism just starts to move
    double maxVel = 300.0;  // [deg/s]

    public CycloidalDrive(final int CANID) {
        setName("CycloidalDrive_" + CANID);
        // set our control constants for pos and vel pids
        velHWPid.setIZone(HW_IZone);
        posPid.setIZone(pos_IZone);
        posPid.setIntegratorRange(-15.0, 15.0);

        servo = new NeoServo(CANID, posPid, velHWPid, false,SparkFlex.class);
        
        // setup servo
        servo  // units should be [deg] and [deg/s]
            .setConversionFactor(CONV_FACTOR )
            .setTolerance(0.5, 3.0)  // [deg], [deg/s]
            .setSmartCurrentLimit(35, 5)  // [amp], [amp]
            .setVelocityHW_PID(maxVel, maxAccel)
            .setMaxVelocity(maxVel)
            .setAFFVelocityComp(true);

        servo.setName(this.getName()+"/NeoServo-55");

        // get refs to servo Spark stuff.
        servo_ctrlr = servo.getController();    
        servo.setPosition(0.0);
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

    public Command cmdCalibrateAtZero() {
        return runOnce(() -> {           
            this.servo.setPosition(0.0);  // sets new zero point
        });
    }

    // Add a DEMO bindings - we don't normally do this but for a demo 
    // it is handy because bot-on-board spec files change frequently
    // as they are used during the season.
    public void setDemoBindings(CommandXboxController xbox) {
        //bindings for Cycloid demo - use POV buttons with new ss cmd pattern        
        //velocity cmds while held it should spin
        xbox.povLeft().whileTrue(this.cmdVelocity(300.0))
                      .onFalse(this.cmdVelocity(0.0));

        xbox.povRight().whileTrue(this.cmdVelocity(-120.0))
                       .onFalse(this.cmdVelocity(0.0));
        
        // Cmd to known points
        xbox.povUp().onTrue(this.cmdPosition(0.0));
        xbox.povDown().onTrue(this.cmdPosition(180.0));
        xbox.y().onTrue(this.cmdCalibrateAtZero());
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
            //motor count based pos/vel
            addEntry("internal_pos", CycloidalDrive.this.servo::getPosition, 2);
            addEntry("internal_vel", CycloidalDrive.this.servo::getVelocity, 2);
            addEntry("at_setpoint", CycloidalDrive.this::atSetpoint);

            // other info about servo's motor
            addEntry("mtr_appliedOutput", CycloidalDrive.this.servo_ctrlr::getAppliedOutput, 2);
            addEntry("mtr_OutputAmps", CycloidalDrive.this.servo_ctrlr::getOutputCurrent, 2);
            addEntry("mtr_Temperature", CycloidalDrive.this.servo_ctrlr::getMotorTemperature, 2);

            // start the servo's NeoWatcher which has most of our stuff
            servo.getWatcher();
        }
    }
}
