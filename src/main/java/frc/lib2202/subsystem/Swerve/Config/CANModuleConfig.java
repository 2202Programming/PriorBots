package frc.lib2202.subsystem.swerve.config;

 // CAN IDs for single corner of the swerve drivetrain

  public class CANModuleConfig {

    public final int CANCODER_ID;
    public final int DRIVE_MOTOR_ID;
    public final int ANGLE_MOTOR_ID;


   /**
   * CANModuleConfig - Stores one swerve module's cancoder and two motor CAN IDs
   * 
   */
    public CANModuleConfig(int CANCODER_ID, int DRIVE_MOTOR_ID, int ANGLE_MOTOR_ID) {
      this.CANCODER_ID = CANCODER_ID;
      this.DRIVE_MOTOR_ID = DRIVE_MOTOR_ID;
      this.ANGLE_MOTOR_ID= ANGLE_MOTOR_ID;
    }
  }