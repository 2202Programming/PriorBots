package frc.timbot.subsystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.timbot.Constants;

public class FlywheelSubsystem extends SubsystemBase {
    
    // was in constants
    final double MIN_SHOOTER_SPEED = 200; // One unit represents one position unit per 100ms
    
    final TalonFX motor1;
    final TalonFX motor2;

    VelocityVoltage m_request;

    double kP = 0.1;
    double kI = 0.0;
    double kD = 0.0;

    // commande velocities
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

    public void setSpeed(double speed) { // [RPM]
        velCmd_m1 = speed;
        velCmd_m2 = speed;
        motor1.setControl(m_request.withVelocity(speed).withFeedForward(0.5));
        motor2.setControl(m_request.withVelocity(speed).withFeedForward(0.5));
    }

    public boolean isAtSpeed(double tolerancePercent) {
        if(velCmd_m1 == 0.0) {
            return true;
        }
        
        return ((Math.abs(vel_m2 - velCmd_m2) / velCmd_m2) <= tolerancePercent); // 1% = 0.01

    }

    public void log() {
        SmartDashboard.putNumber("Mtr_1 Speed", vel_m1);
        SmartDashboard.putNumber("Mtr_2 Speed", vel_m2);

        SmartDashboard.putNumber("Mtr_1 Cmd Speed", velCmd_m1);
        SmartDashboard.putNumber("Mtr_2 Cmd Speed", velCmd_m2);
    }
}
