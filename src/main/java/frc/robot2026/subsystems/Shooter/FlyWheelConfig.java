package frc.robot2026.subsystems.Shooter;

import frc.lib2202.util.PIDFController;

public class FlyWheelConfig {
    public PIDFController hw_pid;       // just holds constants for pid, copyTo()/ copyChangesTo() updates hw
    //only sample values
    public double iMaxAccum=0.0;        // Integral of error[m/s]*[s] = [m]
    public double maxOpenLoopRPM=5600;  // [rpm] typical Neo
    public double gearRatio=1.0;        // [] [gearbox-rot/mtr-in] 
    public boolean inverted=false;      // adjust to spin in correct dir, account for motor/gears
    public double flywheelRadius = 0.05;// [m] ~2[in] in [m]
    public double rampRate=0.0;         // [s] time to max speed, 0 disables
    public int stallAmp=60;             // [Amp] stall current limit
    public int freeAmp=5;               // [Amp] current limit at free run
  };
