package frc.timbot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {
    private TalonFX motor1; // motor closer to the solenoid
    private TalonFX motor2; // motor closer to the output

    public void Flywheel(TalonFX motor1, TalonFX motor2) {
        this.motor1 = motor1;
        this.motor2 = motor2;
    }

    @Override
    public void periodic() {
    }

    public void speed(double speed) {
        motor1.set(speed);
        motor2.set(speed);
    }

    public boolean isAtSpeed() {
        if ((motor1.getVelocity() > 0) && (motor2.getVelocity() > 0)) {
            return true;
        }
        return false;
    }
}
