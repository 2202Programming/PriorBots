package frc.lib2202.subsystem.swerve.config;

import frc.lib2202.util.PIDFController;

public class ChassisConfig {

  // have defaults if no chassis specific PIDF for swerve modules are given
  static PIDFController defaultDrivePIDF = new PIDFController(0.085, 0.00055, 0.0, 0.21292);
  static PIDFController defaultAnglePIDF = new PIDFController(0.01, 0.0, 0.0, 0.0);

  // Kinematics model - wheel offsets from center of robot (0, 0)
  // Left Front given below, symmetry used for others
  public final double XwheelOffset; // meters, half of X wheelbase
  public final double YwheelOffset; // meters, half of Y wheelbase

  public final double wheelCorrectionFactor; // percent
  public final double wheelDiameter; // meters
  public final double kSteeringGR; // [mo-turns to 1 angle wheel turn]
  public final double kDriveGR; // [mo-turn to 1 drive wheel turn]

  // allow Hardware pid configs to be chassis specific too
  public final PIDFController drivePIDF; // drive wheel hw pid consts
  public final PIDFController anglePIDF; // drive angle hw pid consts

  public ChassisConfig(double XwheelOffset, double YwheelOffset, double wheelCorrectionFactor, double wheelDiameter,
      double kSteeringGR, double kDriveGR, PIDFController drivePIDF, PIDFController anglePIDF) {
    
    this.XwheelOffset = XwheelOffset;
    this.YwheelOffset = YwheelOffset;
    this.wheelCorrectionFactor = wheelCorrectionFactor;
    this.wheelDiameter = wheelDiameter * wheelCorrectionFactor;
    this.kSteeringGR = kSteeringGR;
    this.kDriveGR = kDriveGR;
    this.drivePIDF = drivePIDF;
    this.anglePIDF = anglePIDF;

  }

  public ChassisConfig(double XwheelOffset, double YwheelOffset, double wheelCorrectionFactor, double wheelDiameter,
      double kSteeringGR, double kDriveGR) {
    this(XwheelOffset, YwheelOffset, wheelCorrectionFactor, wheelDiameter, kSteeringGR, kDriveGR,
        defaultDrivePIDF, defaultAnglePIDF);
  }
}
