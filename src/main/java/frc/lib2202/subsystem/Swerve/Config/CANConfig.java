package frc.lib2202.subsystem.swerve.config;

 // CAN IDs for swerve drivetrain

  public class CANConfig {

    public final CANModuleConfig FL_MODULE;
    public final CANModuleConfig FR_MODULE;
    public final CANModuleConfig BL_MODULE;
    public final CANModuleConfig BR_MODULE;

   /**
   * CANCONFIG - four CANModuleConfig objects for the four corners
   * 
   */
    public CANConfig(CANModuleConfig FL_MODULE, CANModuleConfig FR_MODULE, CANModuleConfig BL_MODULE, CANModuleConfig BR_MODULE) {
      this.FL_MODULE = FL_MODULE;
      this.FR_MODULE = FR_MODULE;
      this.BL_MODULE = BL_MODULE;
      this.BR_MODULE = BR_MODULE;
    }
  }