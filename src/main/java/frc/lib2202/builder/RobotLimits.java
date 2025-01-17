package frc.lib2202.builder;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

// 1/11/2025 dpl  Use units for robot speeds

public class RobotLimits {
    //Start with reasonable defaults...
    public double motorMaxRPM = 5600; // drive motor limit
    
    // see  https://docs.revrobotics.com/sparkmax/software-resources/configuration-parameters
    public int driveStallAmp = 40; 
    public int angleStallAmp = 20;
    public int freeAmp = 20;

    // Constraints on speeds enforeced in DriveTrain    
    public double kMaxSpeed;            // [m/s] new gears 3/23/24 16.6 m/s max
    public double kMaxAngularSpeed;     // [rad/s]

    public RobotLimits(LinearVelocity maxSpeed, AngularVelocity maxAngularSpeed) {
        kMaxSpeed = maxSpeed.in(Units.MetersPerSecond);
        kMaxAngularSpeed = maxAngularSpeed.in(Units.RadiansPerSecond);      
    }

    public RobotLimits setMaxSpeed(LinearVelocity maxSpeed) {
        kMaxSpeed= maxSpeed.in(Units.MetersPerSecond);
        return this;
    }

    public RobotLimits setAngularSpeed(AngularVelocity maxAngularSpeed) {
        kMaxAngularSpeed = maxAngularSpeed.in(Units.RadiansPerSecond);
        return this;
    }

    public RobotLimits setMaxMotorRPM(double rpm) {
        motorMaxRPM = rpm;
        return this;
    }

    /*
     * passing 0 will keep default
     */
    public RobotLimits setMotorAmps(int free, int driveStall, int angleStall) {
        freeAmp = (free != 0) ? free : freeAmp;
        driveStallAmp = (driveStall != 0) ? driveStall : driveStallAmp;
        angleStallAmp = (angleStall != 0) ? angleStall : angleStallAmp;
        return this;
    }

    /* accessors if we don't want a simple struct
    // robot speed limits
    public double getMotorMaxRPM() {return motorMaxRPM;}
    public double getRobotMaxSpeed() {return kMaxSpeed;}
    public double getRobotMaxAngularSpeed() {return kMaxAngularSpeed;}
    
    // drive/angle motor Amp limits
    int getFreeAmp() {return freeAmp;}
    int getDriveStallAmp() {return driveStallAmp;}
    int getAngleStallAmp() {return angleStallAmp;}
    */

}
