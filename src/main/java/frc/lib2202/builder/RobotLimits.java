package frc.lib2202.builder;
import static frc.lib2202.Constants.MperFT;

public class RobotLimits {
    
    //Start wit reasonable defaults...
    public double motorMaxRPM = 5600; // drive motor limit
    
    // see  https://docs.revrobotics.com/sparkmax/software-resources/configuration-parameters
    public int driveStallAmp = 40; // dpl 3/15 was 30
    public int angleStallAmp = 20;
    public int freeAmp = 20;

    // Constraints on speeds enforeced in DriveTrain    
    public double kMaxSpeed = 16.2 * MperFT; // [m/s] new gears 3/23/24 16.6 m/s max
    public double kMaxAngularSpeed = 2 * Math.PI; // [rad/s]

    public RobotLimits(double maxSpeedFPS, double maxAngularSpeedDPS) {
        kMaxSpeed = maxSpeedFPS * MperFT;
        kMaxAngularSpeed = maxAngularSpeedDPS * Math.PI;        
    }

    public RobotLimits setMaxSpeed(double maxSpeed) {
        kMaxSpeed= maxSpeed;
        return this;
    }

    public RobotLimits setAngularSpeed(double maxAngularSpeed) {
        kMaxAngularSpeed = maxAngularSpeed;
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
