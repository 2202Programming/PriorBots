package frc.timbot.subsystem.Shooter;

import static frc.lib2202.Constants.MperFT;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.PIDFController;

public class Shooter extends SubsystemBase {
    final public IFlyWheel flywheelFront;
    final public IFlyWheel flywheelBack;
    final FlyWheelConfig cfg;
    public double maxVelocity = 20;
  
    public Shooter(int flywheelFrontID, int flywheelBackID) {
        cfg = initFlyWheelConfigCTRE();
        flywheelFront = new FlyWheelCtre(flywheelFrontID, cfg);
        flywheelBack = new FlyWheelCtre(flywheelBackID, cfg);
        this.getWatcherCmd();
    }

        // for testing Kraken
    private FlyWheelConfig initFlyWheelConfigCTRE() {
        double kP = 0.5; //
        double kI = 0.001; // feels kind of bs
        double kD = 0.0; // Seems innsensitive until you add an extremely large value
        double kF = 0.115 *(42.0/30.0); // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V,
                          // 1/8.33 =// 0.12 volts / rotation per second
        double iZone = 0.0; // unused in Talon CTRE controller

        FlyWheelConfig cfg = new FlyWheelConfig();
        cfg.inverted = false;
        cfg.rampRate = 0.0; // not implemented in ctre, but could be
        cfg.gearRatio = 30 / 42; // new kraken pulleys
        cfg.stallAmp = 80; // [amp] Use as stator amps
        cfg.freeAmp = 10; // [amp] //unused
        cfg.maxOpenLoopRPM = 5800.0; // measure at full power or motor spec
        cfg.flywheelRadius = (3.0 / 12.0) * MperFT; // [m] 2 [inch] converted [m]
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
        builder.addDoubleProperty("vel_max", this::getMaxVelocity, this::setMaxVelocity);

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
            //this.flywheelBack.setSetpoint(cmd_vel);
        });
    }

    public Command cmdVelocityFront(double cmd_vel) {
        return runOnce(() -> {
            this.flywheelFront.setSetpoint(cmd_vel);
        });
    }

    public Command cmdVelocityBack(double cmd_vel) {
        return runOnce(() -> {
            this.flywheelBack.setSetpoint(cmd_vel);
        });
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public void setMaxVelocity(double value) {
        maxVelocity = value;
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
            addEntry("velocity_back", Shooter.this.flywheelBack::getVelocity, 2);
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
    

