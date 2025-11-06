package frc.robot2025.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.util.NeoServo;

public class CycloidalDrive extends SubsystemBase {
    private NeoServo servo;

    final double maxVel = 100.0;  // [winch deg/s]
    final double maxAccel = 75.0; // [winch deg/s/s]
    
    double cmdPos;
    double cmdVel;
  
    public CycloidalDrive() {
        // SET CAN ID TO PROPER VALUE
        servo = new NeoServo(99, null, null, false);

        servo  // units should be [deg] and [deg/s]
        .setVelocityHW_PID(maxVel, maxAccel)
        .setMaxVelocity(maxVel);
    }

    @Override
    public void periodic() {
        servo.periodic();
    }

    public void setVelocity(double vel) {
        cmdVel = vel;
        servo.setVelocityCmd(vel);
    }

    public double getVelocity()
    {
        return servo.getVelocity();
    }
}
