package frc.lib2202.util;

import static frc.lib2202.Constants.DT;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * PIDFController - extends current (2020) pidcontroller to include a feed
 * forward gain which is not currently part of the WPILib version.
 * 
 * This is useful for holding values for devices like the talon SRX or sparkMax
 * which may have a feed forward gain or Izone.
 * 
 * 2/16/21 added CopyTo helper functions
 * 
 */
public class PIDFController extends PIDController {
    // hardware refs if used
    SparkPIDController sparkMaxController = null;
    double m_smartMaxVel = 0.1;
    double m_smartMaxAccel = .01;

    double m_Kf = 0.0;
    
    public PIDFController(double Kp, double Ki, double Kd, double Kf) {
        this(Kp, Ki, Kd, Kf, DT);
    }

    public PIDFController(double Kp, double Ki, double Kd, double Kf, double period) {
        super(Kp, Ki, Kd, period);
        setF(Kf);
    }

    public PIDFController(PIDFController src) {
        this(src.getP(), src.getI(), src.getD(), src.getF(), src.getPeriod());
    }

    public void setPIDF(double kP, double kI, double kD, double kF) {
        setPID(kP, kI, kD);
        setF(kF);
    }

    // Accessors for the Kf
    public double getF() {
        return m_Kf;
    }

    public void setF(double Kf) {
        m_Kf = Kf;
    }
    
    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint    The new setpoint of the controller.
     */
    @Override
    public double calculate(double measurement, double setpoint) {
        return super.calculate(measurement, setpoint) + (m_Kf * setpoint);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     */
    @Override
    public double calculate(double measurement) {
        return calculate(measurement, getSetpoint());
    }

    /**
     * Copied from base class and feed forward added.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("f", this::getF, this::setF);
        builder.addDoubleProperty("iZone", this::getIZone, this::setIZone);
    }

    public boolean equals(PIDFController other) {
        return getP() == other.getP() && getI() == other.getI() && getD() == other.getD() && getF() == other.getF();
    }

    /**
     * 
     * copyTo() copies this pid's values down to a hardward PID implementation
     * 
     * @param dest          device
     * @param slot          control slot on device
     * 
     *                      optional smartMax vel and accel limits may be given
     * @param smartMaxVel   optional, 0.1 [units/s]
     * @param smartMaxAccel optional 0.01 [units/s^2]
     */
    public void copyTo(SparkPIDController dest, int slot) {
        copyTo(dest, slot, m_smartMaxVel, m_smartMaxAccel);
    }

    public void copyTo(SparkPIDController dest, int slot, double smartMaxVel, double smartMaxAccel) {
        dest.setP(this.getP(), slot);
        dest.setI(this.getI(), slot);
        dest.setD(this.getD(), slot);
        dest.setFF(this.getF(), slot);
        dest.setIZone(this.getIZone(), slot);
        dest.setSmartMotionMaxVelocity(smartMaxVel, slot);
        dest.setSmartMotionMaxAccel(smartMaxAccel, slot);
        sparkMaxController = dest;
        m_smartMaxVel = smartMaxVel;
        m_smartMaxAccel = smartMaxAccel;
    }

    // compares an updated PIDF with this one and updates it and the hardware
    public void copyChangesTo(SparkPIDController dest, int slot, PIDFController updated) {
        // update pid values that have changed
        if (getP() != updated.getP()) {
            setP(updated.getP());
            dest.setP(getP(), slot);
        }

        if (getI() != updated.getI()) {
            setI(updated.getI());
            dest.setI(getI(), slot);
        }

        if (getD() != updated.getD()) {
            setD(updated.getD());
            dest.setD(getD(), slot);
        }

        if (getF() != updated.getF()) {
            setF(updated.getF());
            dest.setFF(getF(), slot);
        }

        if (getIZone() != updated.getIZone()) {
            setIZone(updated.getIZone());
            dest.setIZone(getIZone(), slot);
        }
    }

}
