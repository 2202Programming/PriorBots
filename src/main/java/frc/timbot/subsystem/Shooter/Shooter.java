package frc.timbot.subsystem.Shooter;

import static frc.lib2202.Constants.MperFT;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.PIDFController;
import frc.timbot.Constants.CAN;

public class Shooter extends SubsystemBase {
    final public IFlyWheel flywheelFront;
    final public IFlyWheel flywheelBack;
    final FlyWheelConfig cfg;
    final boolean inverted;

    public Shooter() {
        this("ctre", 0, 0, true);
    }

    public Shooter(String controllerType, int flywheelFrontID, int flywheelBackID) {
        this(controllerType, flywheelFrontID, flywheelBackID, true);
    }

    public Shooter(String controllerType, int flywheelFrontID, int flywheelBackID, boolean inverted) {
        this.inverted = inverted;
        
        // pick which controller we are using
        if (controllerType.equalsIgnoreCase("ctre")) {
            cfg = initFlyWheelConfigCTRE();

            flywheelFront = new FlyWheelCtre(flywheelFrontID, cfg);
            flywheelBack = new FlyWheelCtre(flywheelBackID, cfg);

        } else if (controllerType.equalsIgnoreCase("multi")) {
            cfg = initMultiFlyWheelConfigREV();
            flywheelFront = new FlyWheelRev(flywheelFrontID, cfg);
            flywheelBack = new FlyWheelRev(flywheelBackID, cfg);
        } else if (controllerType.equalsIgnoreCase("flex")) {
            cfg = initFlyWheelConfigREVFlex();
            flywheelFront = new FlyWheelRevFlex(flywheelFrontID, cfg);
            flywheelBack = new FlyWheelRevFlex(flywheelBackID, cfg);
        } else {
            cfg = initFlyWheelConfigREV();
            flywheelFront = new FlyWheelRev(flywheelFrontID, cfg);
            flywheelBack = new FlyWheelRev(flywheelBackID, cfg);
        }
        this.getWatcherCmd();
    }

    // Setup using NEO1
    private FlyWheelConfig initFlyWheelConfigREV() {
        double kP = 0.01;// 0.005; // tune next
        double kI = 0.00005; // finally stiffen speed with I/D
        double kD = 2.0;// 10.0; // Seems innsensitive until you add an extremely large value
        double kF = 0.315;
        double iZone = 1.0; // setting it to 0.0 seems to 'unlock' it

        FlyWheelConfig cfg = new FlyWheelConfig();
        cfg.inverted = inverted;
        cfg.rampRate = 0.0; // try to soften the startup, zero disables
        cfg.gearRatio = 24.0 / 18.0; // this was measured -- DPL + BG 1/19/26
        cfg.stallAmp = 60; // [amp] Check motor specs for amps
        cfg.freeAmp = 10; // [amp]
        cfg.maxOpenLoopRPM = 5800.0; // measure at full power or motor spec
        cfg.flywheelRadius = (2.0 / 12.0) * MperFT; // [m] 2 [inch] converted [m]
        cfg.iMaxAccum = 0.25;
        // PIDF constant holder for hw
        cfg.hw_pid = new PIDFController(kP, kI, kD, kF, "flywheelPIDF");
        cfg.hw_pid.setIZone(iZone);
        return cfg;
    }

    // Setup using Vortex
    private FlyWheelConfig initFlyWheelConfigREVFlex() {
        // Tuned by XS and AN on production alpha bot shooter
        double kP = 0.019;
        double kI = 0.0003;
        double kD = 7.0;
        double kF = 0.171;
        double iZone = 1.0; // setting it to 0.0 seems to 'unlock' it

        FlyWheelConfig cfg = new FlyWheelConfig();
        cfg.inverted = inverted;
        cfg.rampRate = 0.0; // try to soften the startup, zero disables
        cfg.gearRatio = 50.0 / 24.0; //
        cfg.stallAmp = 90; // [amp] Check motor specs for amps
        cfg.freeAmp = 15; // [amp]
        cfg.maxOpenLoopRPM = 5800.0; // measure at full power or motor spec
        cfg.flywheelRadius = (2.0 / 12.0) * MperFT; // [m] 2 [inch] converted [m]
        cfg.iMaxAccum = 0.25;
        // PIDF constant holder for hw
        cfg.hw_pid = new PIDFController(kP, kI, kD, kF, "flywheelPIDF");
        cfg.hw_pid.setIZone(iZone);
        return cfg;
    }

    // tuning from MultiShooter, also rev Neo
    private FlyWheelConfig initMultiFlyWheelConfigREV() {
        double kP = 0.06; // tune next
        double kI = 0.0001; // finally stiffen speed with I/D
        double kD = 80; // Seems innsensitive until you add an extremely large value
        double kF = 0.57;
        double iZone = 1.0; // setting it to 0.0 seems to 'unlock' it

        FlyWheelConfig cfg = new FlyWheelConfig();
        cfg.inverted = inverted;
        cfg.rampRate = 0.0; // try to soften the startup, zero disables
        cfg.gearRatio = 1.0;
        cfg.stallAmp = 80; // [amp] Check motor specs for amps TESTING 80 FOR MULTI DUE TO HIGH DROP
        cfg.freeAmp = 10; // [amp]
        cfg.maxOpenLoopRPM = 5800.0; // measure at full power or motor spec
        cfg.flywheelRadius = (2.0 / 12.0) * MperFT; // [m] 2 [inch] converted [m]
        cfg.iMaxAccum = 0.25;
        // PIDF constant holder for hw
        cfg.hw_pid = new PIDFController(kP, kI, kD, kF, "flywheelPIDF");
        cfg.hw_pid.setIZone(iZone);
        return cfg;
    }

    // for testing Kraken
    private FlyWheelConfig initFlyWheelConfigCTRE() {
        double kP = 0.7; //
        double kI = 4.0; // feels kind of bs
        double kD = 0.01; // Seems innsensitive until you add an extremely large value
        double kF = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V,
                          // 1/8.33 =// 0.12 volts / rotation per second
        double iZone = 0.0; // unused in Talon CTRE controller

        FlyWheelConfig cfg = new FlyWheelConfig();
        cfg.inverted = inverted;
        cfg.rampRate = 0.0; // not implemented in ctre, but could be
        cfg.gearRatio = 1.0 / 1.0; // new kraken pulleys
        cfg.stallAmp = 80; // [amp] Use as stator amps
        cfg.freeAmp = 10; // [amp] //unused
        cfg.maxOpenLoopRPM = 5800.0; // measure at full power or motor spec
        cfg.flywheelRadius = (2.0 / 12.0) * MperFT; // [m] 2 [inch] converted [m]
        cfg.iMaxAccum = 0.0; // unused in ctre
        // PIDF constant holder for hw
        cfg.hw_pid = new PIDFController(kP, kI, kD, kF, "flywheelPIDF");
        cfg.hw_pid.setIZone(iZone);
        return cfg;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("atVelocity", this::atSetpoint, null);
        builder.addDoubleProperty("vel_cmd_front", flywheelFront::getSetpoint, flywheelFront::setSetpoint);
        builder.addDoubleProperty("vel_cmd_back", flywheelBack::getSetpoint, flywheelBack::setSetpoint);
        builder.addDoubleProperty("vel_measured_front", flywheelFront::getVelocity, null);
        builder.addDoubleProperty("vel_measured_back", flywheelBack::getVelocity, null);
        builder.addDoubleProperty("vel_tolerance_front", flywheelFront::getTolerance, flywheelFront::setVelocityTolerance);
        builder.addDoubleProperty("vel_tolerance_back", flywheelBack::getTolerance, flywheelBack::setVelocityTolerance);

        // Rev Only
        /*if (flywheel instanceof FlyWheelRev) {
            var revfw = (FlyWheelRev) flywheel;
            builder.addDoubleProperty("iMaxAccum", revfw::getIMaxAccum, revfw::setIMaxAccum);
            builder.addDoubleProperty("iAccum", revfw::getIAccum, null);
            builder.addDoubleProperty("iZone", cfg.hw_pid::getIZone, cfg.hw_pid::setIZone);
            builder.addDoubleProperty("ramp_rate", revfw::getRampRate, revfw::setRampRate);
        } */

        // hook in the PID
        cfg.hw_pid.initSendable(builder);
    }

    @Override
    public void periodic() {
        // update hw, only needed if changes to HW_PID - TODO test mode?
        flywheelFront.update_hardware();
        flywheelBack.update_hardware();
    }

    // Add a watcher so we can see stuff on network tables
    public WatcherCmd getWatcherCmd() {
        return this.new ShooterWatcher();
    }

    // Shooter API
    public boolean atSetpoint() {
        /* The other flywheel runs at 80% of the front flywheel's speed,
        therefore when one flywheel's setpoint is 0, the other is aswell
        */
        boolean shooterAtRest = flywheelFront.getSetpoint() == 0.0;
        return flywheelFront.atSetpoint() && flywheelBack.atSetpoint() && !shooterAtRest;
    }

    // Basic Commands
    public Command cmdVelocity(double cmd_vel) {
        return runOnce(() -> {
            this.flywheelFront.setSetpoint(cmd_vel);
            this.flywheelBack.setSetpoint(cmd_vel * 0.8);
        });
    }

    public Command cmdVelocityWait(double cmd_vel) {
        return Commands.sequence(
                cmdVelocity(cmd_vel),
                Commands.waitUntil(this::atSetpoint),
                Commands.print(getName() + " is atSetpoint " + cmd_vel))
                .withName(getName() + ":cmdVelocityWait=" + cmd_vel);
    }

    //use this to run the shooter for a short period of time to wind
    //down
    public Command cmdVelocityDuration(double cmd_vel, double seconds){
        return Commands.sequence(
                cmdVelocity(cmd_vel),
                new WaitCommand(seconds),
                cmdVelocity(0.0));
    }

    // Testing Bindings
    public void setTestBindings(CommandXboxController xbox) {
        xbox.leftTrigger(0.5)
                .whileTrue(this.cmdVelocity(65.0)) // [m/s]
                .onFalse(this.cmdVelocity(0.0));
        xbox.rightTrigger(0.5)
                .whileTrue(this.cmdVelocity(50.0)) // [m/s]
                .onFalse(this.cmdVelocity(0.0));
        xbox.leftBumper()
                .whileTrue(this.cmdVelocity(45.0)) // [m/s]
                .onFalse(this.cmdVelocity(0.0));
        xbox.rightBumper()
                .whileTrue(this.cmdVelocity(30.0)) // [m/s]
                .onFalse(this.cmdVelocity(0.0));

        xbox.b().onTrue(this.cmdVelocity(0.0)); // [m/s]
    }

    // watcher will put values on the network tables for viewing elastic
    class ShooterWatcher extends WatcherCmd {
        ShooterWatcher() {
            addEntry("velocity_front", Shooter.this.flywheelFront::getVelocity, 2);
            addEntry("at_setpoint", Shooter.this::atSetpoint);
            // other info about flywheel's motor
            addEntry("mtr_appliedOutput_front", Shooter.this.flywheelFront::getAppliedOutput, 2);
            addEntry("mtr_appliedOutput_back", Shooter.this.flywheelBack::getAppliedOutput, 2);
            addEntry("mtr_OutputAmps_front", Shooter.this.flywheelFront::getOutputCurrent, 2);
            addEntry("mtr_OutputAmps_front", Shooter.this.flywheelBack::getOutputCurrent, 2);
            addEntry("mtr_RPM_front", Shooter.this.flywheelFront::getMotorRPM, 1);
            addEntry("mtr_RPM_back", Shooter.this.flywheelBack::getMotorRPM, 1);
            addEntry("mtr_Temperature_front", Shooter.this.flywheelFront::getMotorTemperature, 2);
            addEntry("mtr_Temperature_back", Shooter.this.flywheelBack::getMotorTemperature, 2);
        }
    }
}
    

