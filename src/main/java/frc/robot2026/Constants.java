package frc.robot2026;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Meters;

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

    public static final int r_arm = 998; // TODO: CHANGE
    public static final int l_arm = 45;

    // Intake
    public static final int IntakeID = 40; // this canID is correct DONT CHANGE IT
    // public static final int IntakeBottomID = 41;

    // Hopper
    public static final int BeltID = 41;

    // Shooter & Indexer - LEFT and Right Side
    public static final int ShooterIDRight = 50;
    public static final int RIndexerID = 51;
    public static final int ShooterIDLeft = 52;
    public static final int LIndexerID = 53;

    // IMU
    public static final int PIGEON_IMU_CAN = 60;

    // CANRange
    public static final int CANRANGE_R_CAN = 62;
    public static final int CANRANGE_L_CAN = 61;

  }

  public static final class PWM {
    // public static final int Wrist = 0;
  }

  public static final class AnalogIn {
    // public static final int Wrist = 0;
  }

  // pnumatics control module 1
  public static final class PCM1 {
  }

  // pnumatics control module 2
  public static final class PCM2 {
  }

  public final class DigitalIO {
    // TODO -VERIFY all DIO - 2/26/26
    public static final int IntakeGate = 0;
    public static final int IndexerGateRight = 1;
    public static final int IndexerGateLeft = 2;
    public static final int FuelGauge = 3;

  }

  // The Field info use WPILIB data
  public class TheField {
    public static AprilTagFields fieldChoice = AprilTagFields.k2026RebuiltAndymark; // or k2026RebuiltWelded
    public static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(fieldChoice);
  }

  public static class Vision {
    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = TheField.fieldLayout;

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static class FieldConstants {
    /** The longer side, corresponds to X values */
    public static final Distance kFieldLength = Meters.of(16.541);
    /** The shorter side, corresponds to Y values */
    public static final Distance kFieldWidth = Meters.of(8.069);
    /** The length of the alliance zone, corresponds to X axis */
    public static final Distance kAllianceZoneLength = Meters.of(3.8);

    public static final Pose3d kBlueHub = new Pose3d(4.632516, 4.011139, 1.83, Rotation3d.kZero);
    public static final List<Pose2d> kFeedTargets = List.of(
        new Pose2d(2.5, 2, Rotation2d.kZero),
        new Pose2d(2.5, 6.0, Rotation2d.kZero));
  }

}