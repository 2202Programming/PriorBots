// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  

  /*------------------------Drivetrain-------------------------

  Moved to new format for better support of multiple bots -- See IRobotSpec and RobotSpec_<bot>.java
  files for details...   DPL 

  public static final class DriveTrain {
    // motor constraints
    public static final double motorMaxRPM = 5600; // motor limit
    // see
    // https://docs.revrobotics.com/sparkmax/software-resources/configuration-parameters
    public static final int driveStallAmp = 40; // dpl 3/15 was 30
    public static final int angleStallAmp = 20;
    public static final int freeAmp = 20;

    // Constraints on speeds enforeced in DriveTrain
  
    public static final double kMaxSpeed = 16.2 * MperFT; // [m/s] new gears 3/23/24 16.6 m/s max
    public static final double kMaxAngularSpeed = 2 * Math.PI; // [rad/s]
  
    // SmartMax PID values [kp, ki, kd, kff] - these get sent to hardware controller
    // DEBUG - SET FF first for drive, then add KP

    // DriveTrain pid values
    // alpha constant public static final PIDFController drivePIDF = new
    // PIDFController(0.2 * FTperM, 5.0e-6, 0.0, 0.087782 * FTperM);
    //public static final PIDFController drivePIDF =  new PIDFController(0.085, 0.00055, 0.0, 0.21292);
    //static {
    //    drivePIDF.setIZone(0.2); // limit Ki to small region of error to prevent windup.
    //}
   // org 3/23: public static final PIDFController drivePIDF = new PIDFController(0.09 * FTperM, 5.0e-6, 0.0, 0.087782 * FTperM);
   // public static final PIDFController anglePIDF = new PIDFController(0.01, 0.0, 0.0, 0.0); // maybe 1.0,0.0,0.1 from
                                                                                            // SDS sample code?
    /*
     * Settings for different swerve bot chassis
     
    // FOR SWERVEBOT, aka Tim 2.0  
    public static final WheelOffsets swerveBotOffsets = new WheelOffsets(-98.942, 91.33, -177.035, -28.215); //FL BL FR BR
    public static final ChassisConfig swerveBotChassisConfig = new ChassisConfig(10.5 / 12, 10.5 / 12, 0.995,
        99.5 / 1000.0, 12.8, 8.14);

    // FOR 2022 Chad Bot - degrees
    public static final WheelOffsets chadBotOffsets = new WheelOffsets(-175.60, -115.40, -162.15, 158.81);
    public static final ChassisConfig chadBotChassisConfig = new ChassisConfig(MperFT * (21.516 / 12.0) / 2.0,
        MperFT * (24.87 / 12) / 2, 0.995, 99.5 / 1000.0, 12.8, 8.14);

    // For 2023 CompetitionBot - Doof
    public static final WheelOffsets doofBotOffsets = new WheelOffsets(129.03, -83.94, -57.83, 139.38);
    public static final ChassisConfig doofBotChassisConfig = new ChassisConfig(
        MperFT * (23.5 / 12.0) / 2.0, // based on CAD in reference_links
        MperFT * (19.5 / 12.0) / 2.0, // based on CAD in reference_links
        0.999, // scale [] <= 1.0
        MperFT * (4.0 / 12.0), // wheel diameter[m] Comp bot is 4" wheels
        12.8, // confirmed with vince
        6.12,
        new PIDFController(0.085, 0.00055, 0.0, 0.21292),
        new PIDFController(0.01, 0.0, 0.0, 0.0) //angle
        ); // confirmed with vince

    public static final WheelOffsets comp2024AlphaBotOffsets = new WheelOffsets(43.85746387, 24.096825,
        -65.21481, -43.066333125);
    public static final ChassisConfig comp2024AlphaBotChassisConfig = new ChassisConfig(
        MperFT * (25 / 12.0) / 2.0,
        MperFT * (20.75 / 12.0) / 2.0,
        0.957, // scale [] <= 1.0
        MperFT * (4.0 / 12.0),
        21.428,
        6.12,
        new PIDFController(0.085, 0.00055, 0.0, 0.21292), // drive
        new PIDFController(0.01, 0.0, 0.0, 0.0)  // angle
        );
        // finish AlphaBot's drivePIDF 
        static {
          comp2024AlphaBotChassisConfig.drivePIDF.setIZone(0.2);
        }


    // TODO: For 2024 CompetitionBotBeta ***NOT YET CONFIRMED
    // FL: offset 0.0, measured 126.474609375, should be -126.474609375
    // FR: offset 0.0, measured -65.21484375, should be 65.21484375
    // BL: offset 0.0, measured -28.828125, should be 28.828125
    // BR: offset 0.0, measured 115.224609375, should be -115.224609375
    public static final WheelOffsets comp2024BetaBotOffsets = // new WheelOffsets(0.0751953125*180.0,
                                                              // -0.41845703125*180.0, 0.090087890625*180.0,
                                                              // 0.090087890625*180.0);
        new WheelOffsets(-125.595, 28.125, -114.785, -115.752);
    public static final ChassisConfig comp2024BotBetaChassisConfig = new ChassisConfig(
        MperFT * (24.875 / 12.0) / 2.0, // x
        MperFT * (20.5 / 12.0) / 2.0, // y
        0.987, // scale [] <= 1.0
        MperFT * (4.0 / 12.0),
        21.428,
        6.12,
        new PIDFController(0.085, 0.00055, 0.0, 0.21292), // drive
        new PIDFController(0.01, 0.0, 0.0, 0.0)  // angle
        );
        // finish BetaBot's drivePIDF 
        static {
          comp2024BotBetaChassisConfig.drivePIDF.setIZone(0.2);
        }


    public static final ChassisInversionSpecs comp2024BotAlphaInversionSpecs = new ChassisInversionSpecs(
        new ModuleInversionSpecs(true, true, false), // FR
        new ModuleInversionSpecs(false, true, false), // FL
        new ModuleInversionSpecs(true, true, false), // BR
        new ModuleInversionSpecs(false, true, false)); // BL

    public static final ChassisInversionSpecs comp2024BotBetaInversionSpecs = new ChassisInversionSpecs(
        new ModuleInversionSpecs(false, true, false), // FR
        new ModuleInversionSpecs(true, true, false), // FL
        new ModuleInversionSpecs(false, true, false), // BR
        new ModuleInversionSpecs(true, true, false)); // BL

    public static final ChassisInversionSpecs swerveBotChassisInversionSpecs = new ChassisInversionSpecs(
        new ModuleInversionSpecs(true, false, false), // FR
        new ModuleInversionSpecs(false, false, false), // FL
        new ModuleInversionSpecs(true, false, false), // BR
        new ModuleInversionSpecs(false, false, false)); // BL

    public static final ChassisInversionSpecs chadBotChassisInversionSpecs = new ChassisInversionSpecs(
        new ModuleInversionSpecs(true, false, false), // FR
        new ModuleInversionSpecs(false, false, false), // FL
        new ModuleInversionSpecs(true, false, false), // BR
        new ModuleInversionSpecs(false, false, false)); // BL

    public static final ChassisInversionSpecs doofBotChassisInversionSpecs = new ChassisInversionSpecs(
        new ModuleInversionSpecs(true, false, false), // FR
        new ModuleInversionSpecs(false, false, false), // FL
        new ModuleInversionSpecs(true, false, false), // BR
        new ModuleInversionSpecs(false, false, false)); // BL

    // Support for multiple robots on same code base

    public static final ModuleConfig comp2024CAN_FL = new ModuleConfig(29, 24, 25);
    public static final ModuleConfig comp2024CAN_FR = new ModuleConfig(30, 26, 27);
    public static final ModuleConfig comp2024CAN_BL = new ModuleConfig(28, 22, 23);
    public static final ModuleConfig comp2024CAN_BR = new ModuleConfig(31, 20, 21);

    public static final ModuleConfig swerveBotCAN_FL = new ModuleConfig(7, 20, 21);
    public static final ModuleConfig swerveBotCAN_FR = new ModuleConfig(30, 26, 27);
    public static final ModuleConfig swerveBotCAN_BL = new ModuleConfig(28, 22, 23);
    public static final ModuleConfig swerveBotCAN_BR = new ModuleConfig(31, 24, 25);

    public static final CANConfig comp2024BotCANConfig = new CANConfig(comp2024CAN_FL, comp2024CAN_FR, comp2024CAN_BL,
        comp2024CAN_BR);
    public static final CANConfig swerveBotCANConfig = new CANConfig(swerveBotCAN_FL, swerveBotCAN_FR, swerveBotCAN_BL,
        swerveBotCAN_BR);
    public static final CANConfig chadBotCANConfig = new CANConfig(swerveBotCAN_FL, swerveBotCAN_FR, swerveBotCAN_BL,
        swerveBotCAN_BR);
    public static final CANConfig doofBotCANConfig = new CANConfig(swerveBotCAN_FL, swerveBotCAN_FR, swerveBotCAN_BL,
        swerveBotCAN_BR);

  } // end DriveTrain configs

  **************************************************************/
  /*-------------------------Ports/CAN-------------------------------- */
  /**
   * CAN bus IDs
   * 
   * Please keep in order ID order
   * 
   */
  public static final class CAN {
    public static final int ROBORIO = 0;
    public static final int PDP = 1; // for rev
    public static final int PCM1 = 2; // for rev

    // lights
    public static final int CANDLE1 = 3;
    public static final int CANDLE2 = 4;

    // Warning: CAN 7 is used for CANCoder on swerveBot aka Tim 2.0

    // shooter CAN IDs -- MOTORS
    public static final int SHOOTER_L = 15;
    public static final int SHOOTER_R = 16;
    public static final int SHOOTER_ANGLE = 35;

    // Drive Train IDs 20 - 31
    // drive train CAN addresses are set above with CANModuleConfig to support
    // different robots
    // See above CANModuleConfig definitions.
    //
    // Typically: Drv Ang CC Corner
    // -- --- -- ----
    // 20 21 31 BR
    // 22 23 28 BL
    // 24 25 29 FL
    // 26 27 30 FR
    //
    // TODO: Triple check these numbers with controller client softare, CTRE and REV
    // as the numbers differ from comments at end of this file and there seems
    // to be an inconsistent ordering with the CANCoders.
    //
    // There are exceptions, check for your ROBOT.

    // PLACEHOLDERS - use 50 .. 59, max CAN addr is 64
    // Please move to correct location when ID is assigned

    // Intake
    public static final int INTAKE_MTR = 18;
    public static final int ANGLE_MTR = 17;

    // Transfer
    public static final int TRANSFER_MOTOR = 19;

    // Claw
    public static final int CLAW_WHEEL_MOTOR = 16;

    // IMU
    public static final int PIGEON_IMU_CAN = 60;

    // Climber
    public static final int CLIMBER = 36; // palceholder

    //Amp Mechanism
    public static final int AMP_MECHANISM = 10; //find out


    // Whether to burn flash or not
    public static final boolean BURN_FLASH = false; // swerve-mk3
  }

  public static final class PWM{
    public static final int LEFT_AMP_MECHANISM = 1;
    public static final int RIGHT_AMP_MECHANISM = 0;
  }

  public static final class AnalogIn {
    public static final int Pressure_Sensor = 0;
    // public static final int MAGAZINE_ANGLE = 0;
  }

  // pnumatics control module 1
  public static final class PCM1 {
    public static final int Forward = 0;
    public static final int Reverse = 1;
  }

  // pnumatics control module 2
  public static final class PCM2 {
  }

  public final class DigitalIO {
    public static final int Transfer_LightGate = 0;
    public static final int Intake_LightGate = 1;
    public static final int Shooter_HighLimitSwitch = 3; 
    public static final int Shooter_LowLimitSwitch = 2;
  }

  public static final class Transfer_Constants {
    public enum NoteCommandedLocation {
      Transfer, Intake, Swap;
    }

    public static final double TRANSFER_MOTOR_ON = 0.8;
    public static final double TRANSFER_MOTOR_REVERSE = -0.5;
  }

  public static final class Tag_Pose {
    static Rotation3d dummyRot = new Rotation3d();
    // Converted to AprilTag from our deprecated AprilTag2D, z axis and Rotation are fake
    // For a better way to do this look at 2025 or 2026 TheField structure in constants for those years.
    public static final AprilTag ID0 = new AprilTag(0, new Pose3d( 0.0, 0.0, 0.0, dummyRot)); //dont use tag ID 0, placeholder for array
    /**Blue source right */
    public static final AprilTag ID1 = new AprilTag(1, new Pose3d(15.079472, 0.245872, 0.0, dummyRot));
    /**Blue source left */
    public static final AprilTag ID2 = new AprilTag(2, new Pose3d(16.185134, 0.883666, 0.0, dummyRot));
    /**Red speaker right */
    public static final AprilTag ID3 = new AprilTag(3, new Pose3d(16.579342, 4.982718, 0.0, dummyRot));
    /**Red speaker left */
    public static final AprilTag ID4 = new AprilTag(4, new Pose3d(16.579342, 5.547868, 0.0, dummyRot));
    /**Red amp */
    public static final AprilTag ID5 = new AprilTag(5, new Pose3d(14.700758, 8.2042, 0.0, dummyRot));
    /**Blue amp */
    public static final AprilTag ID6 = new AprilTag(6, new Pose3d(1.8415, 8.2042, 0.0, dummyRot));
    /**Blue speaker right */
    public static final AprilTag ID7 = new AprilTag(7, new Pose3d(0.0381, 5.547868, 0.0, dummyRot));
    /**Blue speaker left */
    public static final AprilTag ID8 = new AprilTag(8, new Pose3d(0.0381, 4.982718, 0.0, dummyRot));
    /**Red source right */
    public static final AprilTag ID9 = new AprilTag(9, new Pose3d(0.356108, 0.883666, 0.0, dummyRot));
    /**Red source left */
    public static final AprilTag ID10 = new AprilTag(10,new Pose3d(1.461516, 0.245872, 0.0, dummyRot));
    /**Red stage (counter-clockwse starting at Stage Left) */
    public static final AprilTag ID11 = new AprilTag(11, new Pose3d(11.904726, 3.713226, 0.0, dummyRot));
    public static final AprilTag ID12 = new AprilTag(12, new Pose3d(11.904726, 4.49834, 0.0, dummyRot));
    public static final AprilTag ID13 = new AprilTag(13, new Pose3d(11.220196, 4.105148, 0.0, dummyRot));
    /**Blue state (counter-clockwise starting at Center Stage) */
    public static final AprilTag ID14 = new AprilTag(14, new Pose3d(5.320792, 4.105148, 0.0, dummyRot));
    public static final AprilTag ID15 = new AprilTag(15, new Pose3d(4.641342, 4.49834, 0.0, dummyRot));
    public static final AprilTag ID16 = new AprilTag(16, new Pose3d(4.641342, 3.713226, 0.0, dummyRot));

    public static AprilTag[] tagLocations = {ID0, ID1, ID2, ID3, ID4, ID5, ID6, ID7, ID8, ID9, ID10, ID11, ID12,
                                      ID13, ID14, ID15, ID16};     
  }

  /*-------NT------- */
  public final static class NTStrings {
    public final static String NT_Name_Position = "Position";
  }

}