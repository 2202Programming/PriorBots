package frc.timbot.subsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// The solenoid is able to be turned off, but that shouldn't be something we need to
// consider. According to the wpilib, it has a 200 ms delay before it can be reliable after 
// being turned back on, making it a risky thing to use.
public class FiringSubsystem extends SubsystemBase {
    
    private final DoubleSolenoid m_doubleSolenoid =
    new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);


    // This is for extending the solenoid, which will be used for the purpose of grabbing onto the bottom frisbee. 
    public void extendLoaderSolenoid() {
        m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    // This is for retracting the solenoid, which will be used for the purpose of pulling the frisbee out.
    public void retractLoaderSolenoid() {
        m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    
}
