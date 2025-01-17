/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.chadbot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.chadbot.subsystems.shooter.FlyWheel.FlyWheelConfig;
import frc.chadbot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;
import frc.lib2202.util.PIDFController;
import static frc.lib2202.Constants.MperFT;
import static frc.lib2202.Constants.FTperM;;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * 
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final static String NT_NAME_POSITION = "Position";

  public static final class Autonomous {

    // path coordinates are in meters - utility only works in meters
    public static final Pose2d startPose1 = new Pose2d(7.67, 1.82, new Rotation2d(-180)); // Bottom, furthest from
                                                                                          // terminal
    public static final Pose2d startPose2 = new Pose2d(6.86, 2.63, new Rotation2d(-180)); // Middle
    public static final Pose2d startPose3 = new Pose2d(6.7, 5.47, Rotation2d.fromDegrees(-180)); // Top
    public static final Pose2d hubPose = new Pose2d(8.27, 4.12, new Rotation2d(0)); // Center of Hub
    public static final Pose2d testStartPose = new Pose2d(5, 5, new Rotation2d(-180));  
  }  


  /**
   * CAN bus IDs
   * 
   * Please keep in order ID order
   * 
   * /**
   * CAN bus IDs
   * 
   * Please keep in order ID order
   * 
   */
  public static final class CAN {
    // CAN ID for non-motor devices
    public static final int PDP = 0; // this must be 0
    public static final int PCM1 = 1; // default ID for PCM

    // drive train CANCoders
    public static final int DT_BL_CANCODER = 28;
    public static final int DT_BR_CANCODER = 31;
    public static final int DT_FR_CANCODER = 30;
    public static final int DT_FL_CANCODER = 7;

    // Shooter CAN devices
    public static final int SHOOTER_UPPER_TALON = 10;
    public static final int SHOOTER_LOWER_TALON = 11;

    // Magazine motors
    public static final int MAG_R_SIDE_MTR = 12;
    public static final int MAG_L_SIDE_MTR = 13;
    public static final int MAG_TOP_WHEEL = 16;
    
    // Intake motor
    public static final int INTAKE_MTR = 14;

    // drive train drive / angle motors - sparkmax neo
    public static final int DT_FL_DRIVE = 20;
    public static final int DT_FL_ANGLE = 21;
    public static final int DT_BL_DRIVE = 22;
    public static final int DT_BL_ANGLE = 23;
    public static final int DT_BR_DRIVE = 24;
    public static final int DT_BR_ANGLE = 25;
    public static final int DT_FR_DRIVE = 26;
    public static final int DT_FR_ANGLE = 27;

    // Climber Arms
    public static final int CMB_LEFT_Extend = 34;
    public static final int CMB_RIGHT_Extend = 35;
    public static final int CMB_LEFT_Rotate = 36;
    public static final int CMB_RIGHT_Rotate = 37;

    //IMU
    public static final int PIGEON_IMU_CAN = 60;

    // Whether to burn flash or not
    public static final boolean BURN_FLASH = false; //swerve-mk3
  }

  // PWM assignments on the Rio
  public static final class PWM {
    //dpl unused public static final int INTAKE = 0;
  }

  // Digital IO on the RIO
  public static final class DigitalIO {
    public static final int INTAKE_GATE = 0;
    public static final int MAGAZINE_GATE1 = 1;
    public static final int MAGAZINE_GATE2 = 2;
    public static final int MAGAZINE_GATE3 = 3;
  }

  public static final class AnalogIn {
    // public static final int MAGAZINE_ANGLE = 0;
 }

  // PWM assignments on the Rio
  public static final class PCM1 {
    // Double Solenoid
    public static final int INTAKE_UP_SOLENOID_PCM = 2; // test value
    public static final int INTAKE_DOWN_SOLENOID_PCM = 3; // test value
    public static final int POSITIONER_UP_SOLENOID_PCM = 0; // test value
    public static final int POSITIONER_DOWN_SOLENOID_PCM = 1; // test value
  }

  // if we use a second PCM
  public static final class PCM2 { }


  public static final class ClimbSettings {
    // Hardware Controller constants for velocity and position modes, each gets hw
    // slot
    public static PIDFController rotatePID_vel = new PIDFController(0.0006, 0.00001, 0.1, 0.00384); // [deg/s] - slot 0
    public static PIDFController rotatePID_pos = new PIDFController(0.00, 0.00, 0.00, 0.00384); // [deg] - slot 1
    public static PIDFController extendPID_vel = new PIDFController(0.015, 0.00019, 0.08, 0.08); // [in/s] - slot 0
    public static PIDFController extendPID_pos = new PIDFController(0.05, 0.0, 0.0, 0.0); // [in] - slot 1

    //ext pid ki was .00015

    // Position/vel tolerance for outer position loops
    public static final double TOLERANCE_EXT = .30; // [in]
    public static final double TOLERANCE_EXT_VEL = 0.18; // [in/s]
    public static final double TOLERANCE_ROT = 2.0; // [deg]
    public static final double TOLERANCE_ROT_RATE = 3.0; // [deg/s]
    
    //limit integrator windup (default =1.0)
    public static final double ROT_INTEGRATOR_MIN = 0.0; // [deg/s]
    public static final double ROT_INTEGRATOR_MAX = 5.0; // [deg/s]
    

    // Software outer loop rate limits
    public static final double MAX_VELOCITY_EXT = 10; // [in/s] 100% DutyCycle at 13.5
    public static final double MAX_VELOCITY_ROT = 40; // [deg/s]

    // SmartCurrent limit for brushless
    public static final int MAX_EXT_AMPS = 25;
    public static final int MAX_ROT_AMPS = 25;

    // max angle delta before kill
    public static final double KILL_COUNT = 30;
  }


      //Intake Constants
      public static final class Intake {
        // PID values to get copied to the hardware
        public static PIDFController r_side_mtrPIDF = new PIDFController(1.0, 0.0, 0.0, 0.0);  
        public static PIDFController l_side_mtrPIDF = new PIDFController(1.0, 0.0, 0.0, 0.0); 
      }
      

      //Driver Preferences
      public static final class DriverPrefs {
          public static final double VelExpo = 0.3;        // non-dim [0.0 - 1.0]
          public static final double RotationExpo = 0.9;   // non-dim [0.0 - 1.0]
          public static final double StickDeadzone = 0.05; // non-dim [0.0 - 1.0]
      }

    

    // CANCoder offsets for absolure calibration - stored in the magnet offset of the CC. [degrees]  
    public static final class WheelOffsets{
        public final double CC_FL_OFFSET;
        public final double CC_BL_OFFSET;
        public final double CC_FR_OFFSET;
        public final double CC_BR_OFFSET;

        public WheelOffsets(double FL, double BL, double FR, double BR){
          this.CC_FL_OFFSET = FL;
          this.CC_BL_OFFSET = BL;
          this.CC_FR_OFFSET = FR;
          this.CC_BR_OFFSET = BR;
        }
    }

    public static final class DriveTrain {
        // NTs
        public static final String NT_NAME_DT = "DT"; // expose data under DriveTrain table

        // motor constraints
        public static final double motorMaxRPM = 5600;    // motor limit

        // Constraints on speeds enforeced in DriveTrain
        public static final double kMaxSpeed = 12.0*MperFT; // [m/s]
        public static final double kMaxAngularSpeed = 2*Math.PI; // [rad/s] 

        /****
         * ### REMINDER - enable these once we have basics working
        // Other constraints
        public static final int smartCurrentMax = 60;  //amps in SparkMax, max setting
        public static final int smartCurrentLimit = 35; //amps in SparkMax, inital setting
        */
        // Acceleration limits
        ///public static final double slewRateMax = 2;      //sec limits adjusting slewrate 
        //public static final boolean safetyEnabled = false; 

        // SmartMax PID values [kp, ki, kd, kff] - these get sent to hardware controller
        // DEBUG - SET FF first for drive, then add KP
        
        //public static final PIDFController drivePIDF = new PIDFController(0.09*MperFT, 0.0, 0.0, 0.08076*MperFT);  
        public static final PIDFController drivePIDF = new PIDFController(0.09*FTperM, 0.0, 0.0, 0.08076*FTperM);  
        public static final PIDFController anglePIDF = new PIDFController(0.01, 0.0, 0.0, 0.0); //maybe 1.0,0.0,0.1 from SDS sample code?
        
        
        
      
        

       
    } 
    
    public final static class NTStrings {
      public final static String NT_Name_Position = "Position";
    }

    public final static class MagazineSettings {
      public final static double defaultFrontIntakeSpeed = 0.5; 
      public final static double defaultSideIntakeSpeed = 0.3; 
      public final static double defaultMagazineSpeed = 1.0;
    }

    public static final class Shooter {
      public static final double DefaultRPMTolerance = .05;  // percent of RPM
      public static final ShooterSettings DefaultSettings = new ShooterSettings(20.0, -20.0);  //ft/s, rot/s

      // Power Cell info
      // public static final double PowerCellMass = 3.0 / 16.0; // lbs
      public static final double PCNominalRadius = 10 / 2.0 / 12.0; // feet - power cell
      public static final double PCEffectiveRadius = 8 / 2.0 / 12.0; // feet - compressed radius
      
      public static final double shortVelocity = 40;
      public static final double shortMediumVelocity = 44;
      public static final double mediumVelocity = 50;
      public static final double longVelocity = 60;
      public static final double autoVelocity = 46;

      // limelight distance constants
        // how many degrees back is your limelight rotated from perfectly vertical?
      public static final double LL_MOUNT_ANGLE_DEG = 28.0;
        // distance from the center of the Limelight lens to the floor
      public static final double LL_LENS_HEIGHT_INCHES = 27.0;
        // distance from the target to the floor
      public static final double GOAL_HEIGHT_TO_FLOOR_INCHES = 104.0;
        // adjustment from edge to centre of target
      public static final double EDGE_TO_CENTER_INCHES = 24.0;
        // adjustment factor
      public static final double METERS_TO_INCHES = 39.37;


      public static final double limelight_default_p = 7; //was 4  // [deg/s  / deg-err]
      public static final double limelight_default_i = 0.1;
      public static final double limelight_default_d = 0.1;


      // constraints
      public static final double kMaxFPS = 80;      //max FPS
      public static final double maxLongRage = 8; //maximum range in long distance shooting mode
      public static final double minLongRange = 1.8; //minimum range in long distance shooting mode
      public static final double maxShortRange = 2; //maximum range in short distance shooting mode
      public static final double degPerPixel = 59.6 / 320; //limelight conversion
      public static final double angleErrorTolerance = 2.0; // [deg] allowed angle error to shoot in guided shooting modes
      public static final double angleVelErrorTolerance = 1.0; // [deg/s] allowed angle error to shoot in guided shooting modes
      
      // Flywheel info
      // Flywheel maxOpenLoopRPM and gear ratio are used to calculate kFF in shooter
      public static FlyWheelConfig upperFWConfig = new FlyWheelConfig();
      static {
        upperFWConfig.maxOpenLoopRPM = 4000;  // estimated from 2000 RPM test
        upperFWConfig.gearRatio = 1.0;         // upper encoder:fw is 1:1 
        upperFWConfig.sensorPhase = true;
        upperFWConfig.inverted = false;
        upperFWConfig.flywheelRadius = 2 / 12.0; // feet
        upperFWConfig.pid = new PIDFController(0.12, 0.0, 4.0, 0.034); // kP kI kD kFF
        upperFWConfig.pid.setIZone(1800);
      }

      public static FlyWheelConfig lowerFWConfig = new FlyWheelConfig();
      static {
        lowerFWConfig.maxOpenLoopRPM = 4000;
        lowerFWConfig.gearRatio = 1.0;         // lower encoder:fw is 1:1
        lowerFWConfig.sensorPhase = false;
        lowerFWConfig.inverted = false; 
        lowerFWConfig.flywheelRadius = 2 / 12.0;   //feet 
        lowerFWConfig.pid = new PIDFController(0.12, 0.0, 4.0, 0.034); // kP kI kD kFF
        lowerFWConfig.pid.setIZone(1800);
      }

    }

    public static final class Sensors {
      public enum EncoderID {
        BackLeft, BackRight, FrontLeft, FrontRight
      }
    }
}