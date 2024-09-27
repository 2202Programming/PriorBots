package frc.chadbot.subsystems;

import static frc.chadbot.Constants.CAN;
import static frc.chadbot.Constants.PCM1;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Positioner_Subsystem extends SubsystemBase {
    /**
     * Intake arm consists of a deploy/retractable arm, controlled by Double Solenoid
     * Intake itself consists of a wheel, controlled by Spark PWM value
     */

    //Localized Constants - what valve value does what action
    static final Value UPWARD = Value.kReverse;
    static final Value DOWNWARD = Value.kForward;
    //Instantiations
    final DoubleSolenoid positioner_solenoid = new DoubleSolenoid(CAN.PCM1,
                PneumaticsModuleType.CTREPCM,
                PCM1.POSITIONER_UP_SOLENOID_PCM,
                PCM1.POSITIONER_DOWN_SOLENOID_PCM);

    //Constructor
    public Positioner_Subsystem(){
        
    }
    //Deploy arm mechanism using a Double Solenoids
    public void deploy() {
        positioner_solenoid.set(UPWARD);
    }

    //Retract arm mechanism using a Double Solenoids
    public void retract() {
        positioner_solenoid.set(DOWNWARD);
    }

    //Returns the state of the Intake Arm
    public boolean isDeployed() {
      return ( positioner_solenoid.get() == UPWARD); 
    }

}
