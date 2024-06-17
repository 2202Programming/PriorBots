package frc.lib2202.subsystem.swerve.config;

 // CANCoder offsets for absolure calibration - stored in the magnet offset of
  // the CC. [degrees]
  public class WheelOffsets {
    public final double CC_FL_OFFSET;
    public final double CC_BL_OFFSET;
    public final double CC_FR_OFFSET;
    public final double CC_BR_OFFSET;

    public WheelOffsets(double FL, double BL, double FR, double BR) {
      this.CC_FL_OFFSET = FL;
      this.CC_BL_OFFSET = BL;
      this.CC_FR_OFFSET = FR;
      this.CC_BR_OFFSET = BR;
    }
  }