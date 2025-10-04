package frc.robot2025;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

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
    public static final int CANDLE3 = 5;
    public static final int CANDLE4 = 6;

    // SDT
    // synced as of 1/25/25 
    // https://docs.google.com/spreadsheets/d/1CyHzJscPIuvs0eFUY_qruQcuFui_w2nIXeXUyMwRKBU
    //
    public static final int BL_Angle = 20;
    public static final int BL_Drive = 21;
    public static final int BL_CANCoder = 28;

    public static final int FL_Angle = 23;
    public static final int FL_Drive = 22;
    public static final int FL_CANCoder = 29;

    public static final int BR_Angle = 25;
    public static final int BR_Drive = 24;
    public static final int BR_CANCoder = 31;

    public static final int FR_Angle = 26;
    public static final int FR_Drive = 27;
    public static final int FR_CANCoder = 30;

    public static final int ELEVATOR_MAIN = 40; 
    public static final int ELEVATOR_FOLLOW = 41; 
    public static final int WRIST = 44; 
    public static final int END_EFFECTOR = 43; 

    // Ground Intake
    public static final int IntakeTop = 51;
    public static final int IntakeBtm = 52; 
    public static final int IntakeWheel = 50; 

    public static final int CLIMBER = 55; 
    // IMU
    public static final int PIGEON_IMU_CAN = 60;
  }

  public static final class PWM{
    public static final int Wrist = 0;
  }

  public static final class AnalogIn {
    public static final int Wrist = 0;
  }

  // pnumatics control module 1
  public static final class PCM1 {
  }

  // pnumatics control module 2
  public static final class PCM2 {
  }

  public final class DigitalIO {
    public static final int EndEffector_Lightgate = 2;
    public static final int GroundIntakeHasCoral = 0; // limitswitch
    public static final int GroundIntakeHasAlgae = 1; // lightgate
    public static final int END_EFFECTOR_WHEEL_LOW_LIGHTGATE = 2; 
    public static final int END_EFFECTOR_LOAD_HIGH_LIGHTGATE = 3; 
    public static final int ElevatorZeroLS = 4;    
    
    public static final int SignalLight1 = 7;
    public static final int SignalLight2 = 8;
    public static final int SignalLight3 = 9;              
  }

  //The Field info use WPILIB data
  public class TheField {
    public static AprilTagFields fieldChoice = AprilTagFields.k2025ReefscapeAndyMark; // k2025ReefscapeWelded;
    public static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(fieldChoice);

    // reef tags in order Drivers refer to them, aka driver order
    public static int[] ReefIdsRed =  {11, 10,  9,  8,  7,  6};
    public static int[] ReefIdsBlue = {20, 21, 22, 17, 18, 19};

    public static int[] PickupIdsBlue = {12, 13}; //right, left as viewed from driver stn
    public static int[] PickupIdsRed = {2, 1};    //right, left as viewed from red driver stn

    public static int DeliverAlgaeBlue = 16;
    public static int DeliverAlgaeRed = 3;
  }  
}