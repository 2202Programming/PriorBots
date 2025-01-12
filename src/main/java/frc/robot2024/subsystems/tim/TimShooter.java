package frc.robot2024.subsystems.tim;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TimShooter extends SubsystemBase {

    // this sets the can id for the compressor to 0. (andcreates the compressor)
    private final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0); // creates the solenide

    public TimShooter() {
        //configure compressor on/off points
        m_compressor.enableAnalog(70, 120);// tunrs on the compressor control
       
        //TODO @ Will, create a command that uses the subsystem and ties to a button
        //m_solenoid.set(m_stick.getRawButton(kSolenoidButton)); // code to make the solenoid tdo things
    }

    // expose an API for moving the solnoid
    public void extend(){
        m_solenoid.set(true);
    }
    public void retract(){
        m_solenoid.set(true);
    }
    

}