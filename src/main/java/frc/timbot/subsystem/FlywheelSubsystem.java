package frc.timbot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.timbot.Constants;

public class FlywheelSubsystem extends SubsystemBase {

    private TalonFX motor1;
    private TalonFX motor2;

    VelocityVoltage m_request;

    private double kP = 0.1;
    private double kI = 0.0;
    private double kD = 0.0;

    private double motor1RequestedVelocity;
    private double motor2RequestedVelocity;
    
    public void Flywheel() {
        motor1 = new TalonFX(Constants.CAN.FLYWHEEL_TALON1);
        motor2 = new TalonFX(Constants.CAN.FLYWHEEL_TALON2);

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
        log();
    }

    public void speed(double speed) { // [rot/s]
        motor1RequestedVelocity = speed;
        motor2RequestedVelocity = speed;
        motor1.setControl(m_request.withVelocity(speed).withFeedForward(0.5));
        motor2.setControl(m_request.withVelocity(speed).withFeedForward(0.5));
    }

    public boolean isAtSpeed(double tolerancePercent) {
        return ((Math.abs(getVelocityMotorOne() - motor1RequestedVelocity) / motor1RequestedVelocity) < tolerancePercent); // 1% = 0.01
    }

    public double getVelocityMotorOne() {
        var velocity = motor1.getVelocity();
        return velocity.refresh().getValueAsDouble();
    }

    public double getVelocityMotorTwo() {
        var velocity = motor2.getVelocity();
        return velocity.refresh().getValueAsDouble();
    }

    public void log() {
        double motor1Velocity = getVelocityMotorOne();
        double motor2Velocity = getVelocityMotorTwo();

        SmartDashboard.putNumber("Motor 1 Speed", motor1Velocity);
        SmartDashboard.putNumber("Motor 2 Speed", motor2Velocity);

        SmartDashboard.putNumber("Motor 1 Requested Speed", motor1RequestedVelocity);
        SmartDashboard.putNumber("Motor 2 Requested Speed", motor2RequestedVelocity);
    }
}
