package frc.timbot.subsystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.timbot.Constants;

// TODO: 
// - Implement conversion factor C2026 Flywheel CTRE
// - Add PID Values to Elastic as modifiable
// - Tune motor

public class FlywheelSubsystem extends SubsystemBase {
    
    // was in constants
    final double MIN_SHOOTER_SPEED = 200; // One unit represents one position unit per 100ms
    
    final TalonFX motor1;
    final TalonFX motor2;

    VelocityVoltage m_request;

    // TODO: Tune 
    double kP = 0.75;
    double kI = 0.0;
    double kD = 0.0;

    // command velocities
    double velCmd_m1;
    double velCmd_m2;

    // measured velocities
    double vel_m1;
    double vel_m2;

    final StatusSignal<AngularVelocity> ss_velocity_m1;
    final StatusSignal<AngularVelocity> ss_velocity_m2;
    
    public FlywheelSubsystem() {
        motor1 = new TalonFX(Constants.CAN.FLYWHEEL_TALON1);
        motor2 = new TalonFX(Constants.CAN.FLYWHEEL_TALON2);

        ss_velocity_m1 = motor1.getVelocity();
        ss_velocity_m2 = motor2.getVelocity();

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;

        motor1.getConfigurator().apply(slot0Configs);
        motor2.getConfigurator().apply(slot0Configs);

        m_request = new VelocityVoltage(0).withSlot(0);
    }

    @Override
    public void periodic() {
        vel_m1 = ss_velocity_m1.refresh().getValueAsDouble();
        vel_m2 = ss_velocity_m2.refresh().getValueAsDouble();
        log();
    }

    // Speed is RPS of Motor
    public void setSpeed(double speed1, double speed2) {
        velCmd_m1 = speed1;
        velCmd_m2 = speed2;
        motor1.setControl(m_request.withVelocity(speed1).withFeedForward(0.1));
        motor2.setControl(m_request.withVelocity(speed2).withFeedForward(0.1));
    }

    public void setSpeed(double speed) { // [RPM]
        this.setSpeed(speed, speed);
    }

    public boolean isAtSpeed(double tolerancePercent) {
        if(velCmd_m1 == 0.0 && velCmd_m2 == 0.0) {
            return true;
        }
        
        return ((Math.abs(vel_m2 - velCmd_m2) / velCmd_m2) <= tolerancePercent); // 1% = 0.01

    }

    public boolean isAtSpeed () {
        return isAtSpeed(0.01);
    }

    public double getCmdSpeed () {
        return velCmd_m1;
    }
    public boolean isAtShootSpeed () {
        return isAtSpeed() && getCmdSpeed() > MIN_SHOOTER_SPEED;
    }

    public void log() {
        SmartDashboard.putNumber("Mtr_1 Speed", vel_m1);
        SmartDashboard.putNumber("Mtr_2 Speed", vel_m2);

        SmartDashboard.putBoolean(getName() + "At speed", isAtShootSpeed());

        SmartDashboard.putNumber("Mtr_1 Cmd Speed", velCmd_m1);
        SmartDashboard.putNumber("Mtr_2 Cmd Speed", velCmd_m2);
    }

    public Command cmdVelocity(double cmd_vel1, double cmd_vel2) {
        return runOnce(() -> {
            this.setSpeed(cmd_vel1, cmd_vel2);
        });
    }

    public Command cmdVelocity(double cmd_vel) {
        return runOnce(() -> {
            this.setSpeed(cmd_vel);
        });
    }

    public Command cmdVelocityWait(double cmd_vel1, double cmd_vel2) {
        return Commands.sequence(
                cmdVelocity(cmd_vel1, cmd_vel2),
                Commands.waitUntil(this::isAtShootSpeed),
                Commands.print(getName() + "is at Setpoint" + cmd_vel1 + ", " + cmd_vel2))
                .withName(getName() + ":cmdVelocityWait=" + cmd_vel1 + cmd_vel2);
    }
}
