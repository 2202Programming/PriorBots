// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib2202.subsystem.swerve.config.CANConfig;
import frc.lib2202.subsystem.swerve.config.CANModuleConfig;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ChassisInversionSpecs;
import frc.lib2202.subsystem.swerve.config.ModuleInversionSpecs;
import frc.lib2202.subsystem.swerve.config.WheelOffsets;
import frc.lib2202.util.PIDFController;

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
  // nominal system frame rate, DT (delta T)
  public static final double DT = 0.02; // [s] 20ms framerate 50Hz

  // Handy feet to meters
  public static final double FTperM = 3.28084;
  public static final double MperFT = 1.0 / FTperM;
  public static final double DEGperRAD = 57.3;
  public static final int NEO_COUNTS_PER_REVOLUTION = 42;

  /*------------------------Drivetrain-------------------------*/
  public static final class DriveTrain {
    // motor constraints
    public static final double motorMaxRPM = 5600; // motor limit
    // see
    // https://docs.revrobotics.com/sparkmax/software-resources/configuration-parameters
    public static final int driveStallAmp = 40; // dpl 3/15 was 30
    public static final int angleStallAmp = 20;
    public static final int freeAmp = 20;

    // Constraints on speeds enforeced in DriveTrain
    // TODO make kMaxSpeed Bot dependent
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
     */
    // FOR SWERVEBOT, aka Tim 2.0
    public static final WheelOffsets swerveBotOffsets = new WheelOffsets(-98.942, 91.33, -177.035, -28.215);
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

    public static final CANModuleConfig comp2024CAN_FL = new CANModuleConfig(29, 24, 25);
    public static final CANModuleConfig comp2024CAN_FR = new CANModuleConfig(30, 26, 27);
    public static final CANModuleConfig comp2024CAN_BL = new CANModuleConfig(28, 22, 23);
    public static final CANModuleConfig comp2024CAN_BR = new CANModuleConfig(31, 20, 21);

    public static final CANModuleConfig swerveBotCAN_FL = new CANModuleConfig(7, 20, 21);
    public static final CANModuleConfig swerveBotCAN_FR = new CANModuleConfig(30, 26, 27);
    public static final CANModuleConfig swerveBotCAN_BL = new CANModuleConfig(28, 22, 23);
    public static final CANModuleConfig swerveBotCAN_BR = new CANModuleConfig(31, 24, 25);

    public static final CANConfig comp2024BotCANConfig = new CANConfig(comp2024CAN_FL, comp2024CAN_FR, comp2024CAN_BL,
        comp2024CAN_BR);
    public static final CANConfig swerveBotCANConfig = new CANConfig(swerveBotCAN_FL, swerveBotCAN_FR, swerveBotCAN_BL,
        swerveBotCAN_BR);
    public static final CANConfig chadBotCANConfig = new CANConfig(swerveBotCAN_FL, swerveBotCAN_FR, swerveBotCAN_BL,
        swerveBotCAN_BR);
    public static final CANConfig doofBotCANConfig = new CANConfig(swerveBotCAN_FL, swerveBotCAN_FR, swerveBotCAN_BL,
        swerveBotCAN_BR);

  } // end DriveTrain configs

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
    public static final Translation2d ID0 = new Translation2d(0,0); //dont use tag ID 0, placeholder for array
    /**Blue source right */
    public static final Translation2d ID1 = new Translation2d(15.079472, 0.245872);
    /**Blue source left */
    public static final Translation2d ID2 = new Translation2d(16.185134, 0.883666);
    /**Red speaker right */
    public static final Translation2d ID3 = new Translation2d(16.579342, 4.982718);
    /**Red speaker left */
    public static final Translation2d ID4 = new Translation2d(16.579342, 5.547868);
    /**Red amp */
    public static final Translation2d ID5 = new Translation2d(14.700758, 8.2042);
    /**Blue amp */
    public static final Translation2d ID6 = new Translation2d(1.8415, 8.2042);
    /**Blue speaker right */
    public static final Translation2d ID7 = new Translation2d(0.0381, 5.547868);
    /**Blue speaker left */
    public static final Translation2d ID8 = new Translation2d(0.0381, 4.982718);
    /**Red source right */
    public static final Translation2d ID9 = new Translation2d(0.356108, 0.883666);
    /**Red source left */
    public static final Translation2d ID10 = new Translation2d(1.461516, 0.245872);
    /**Red stage (counter-clockwse starting at Stage Left) */
    public static final Translation2d ID11 = new Translation2d(11.904726, 3.713226);
    public static final Translation2d ID12 = new Translation2d(11.904726, 4.49834);
    public static final Translation2d ID13 = new Translation2d(11.220196, 4.105148);
    /**Blue state (counter-clockwise starting at Center Stage) */
    public static final Translation2d ID14 = new Translation2d(5.320792, 4.105148);
    public static final Translation2d ID15 = new Translation2d(4.641342, 4.49834);
    public static final Translation2d ID16 = new Translation2d(4.641342, 3.713226);

    public static Translation2d[] tagLocations = {ID0, ID1, ID2, ID3, ID4, ID5, ID6, ID7, ID8, ID9, ID10, ID11, ID12,
                                      ID13, ID14, ID15, ID16};

  }

  /*-------NT------- */
  public final static class NTStrings {
    public final static String NT_Name_Position = "Position";
  }

  // public static final class CAN{
  // /* 1/20/24
  // * CAN IDs:
  // * Corner 3 Drive Motor BL: 22
  // * Corner 3 Direction Motor BL: 23
  // * BL Encoder: 28
  // *
  // * Corner 2 Drive Motor FL: 20
  // * Corner 2 Direction Motor FL: 21
  // * FL Encoder: 29
  // *
  // * Corner 4 Drive Motor BR: 24
  // * Corner 4 Direction Motor BR: 25
  // * BR Encoder
  // *
  // * Corner 1 Direction Motor FR: 27
  // * Corner 1 Drive Motor FR: 26
  // * FR Encoder: 30
  // */

  // // CAN IDs for DT encoders
  // public static final int DT_BL_ENCODER = 28;
  // public static final int DT_FL_ENCODER = 29;
  // public static final int DT_FR_ENCODER = 30;
  // public static final int DT_BR_ENCODER = 31;

  // // CAN IDs for drivetrain motors
  // public static final int DT_FL_DRIVE = 20;
  // public static final int DT_FL_DIRECTION = 21;

  // public static final int DT_BL_DRIVE = 22;
  // public static final int DT_BL_DIRECTION = 23;

  // public static final int DT_BR_DRIVE = 24;
  // public static final int DT_BR_DIRECTION = 25;

  // public static final int DT_FR_DRIVE = 26;
  // public static final int DT_FR_DIRECTION = 27;

}