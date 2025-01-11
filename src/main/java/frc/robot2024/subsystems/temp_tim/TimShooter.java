import edu.wpi.first.wpilibj.Compressor;

private final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);/// this sets the can id for the compressor to 0. (and creates the compressor)

private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0); //creates the solenide 

m_compressor.enabledigital(70, 120);//turns on the compressor control

m_solenoid.set(m_stick.getRawButton(kSolenoidButton)); //code to make the solenoid tdo things

