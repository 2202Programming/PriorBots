package frc.lib2202.subsystem.swerve.config;

/*
 * 
 * Everything needed for a single Swerve Module
 * 
 */
public class ModuleConfig {
  public enum CornerID { FrontLeft, FrontRight, BackLeft, BackRight}

  public final CornerID id;

  // CAN ID for a swerve module
  public final int CANCODER_ID;
  public final int DRIVE_MOTOR_ID;
  public final int ANGLE_MOTOR_ID;

  // motor inversions, if needed use setInversions()
  public boolean kDriveMotorInvert = false;
  public boolean kAngleMotorInvert = false;
  public boolean kAngleCmdInvert = false;

  // wheel offsets for CANCoder
  public final double kWheelOffset;

  /**
   * CANModuleConfig - Stores one swerve module's cancoder and two motor CAN IDs
   * 
   */
  public ModuleConfig(CornerID id, int CANCODER_ID, int DRIVE_MOTOR_ID, int ANGLE_MOTOR_ID, double wheelOffset) {
    this.id = id;
    this.CANCODER_ID = CANCODER_ID;
    this.DRIVE_MOTOR_ID = DRIVE_MOTOR_ID;
    this.ANGLE_MOTOR_ID = ANGLE_MOTOR_ID;
    this.kWheelOffset = wheelOffset;
  }

  /**
   * Constructs a ModuleInversionSpecs
   *
   * @param kDriveMotorInvert drive motor invert boolean flag
   * @param kAngleMotorInvert angle motor invert boolean flag
   * @param kAngleCmdInvert   invert angle encoder
   */
  public ModuleConfig setInversions(boolean kDriveMotorInvert, boolean kAngleMotorInvert, boolean kAngleCmdInvert) {
    this.kDriveMotorInvert = kDriveMotorInvert;
    this.kAngleMotorInvert = kAngleMotorInvert;
    this.kAngleCmdInvert = kAngleCmdInvert;
    return this;
  }

  

}