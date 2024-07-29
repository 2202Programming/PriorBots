// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.subsystem;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.Constants;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.LimelightHelpers.LimelightTarget_Fiducial;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight_Subsystem. */

  private NetworkTable table;
  private NetworkTable outputTable;
  //private NetworkTableEntry tx;
  //private NetworkTableEntry ty;
  //private NetworkTableEntry ta;
  //private NetworkTableEntry tv;

  private NetworkTableEntry leds;
  private NetworkTableEntry booleanLeds;
  private NetworkTableEntry NT_hasTarget;
  private NetworkTableEntry nt_bluepose_x;
  private NetworkTableEntry nt_bluepose_y;
  private NetworkTableEntry outputTx;
  private NetworkTableEntry outputTv;
  private NetworkTableEntry pipelineNTE;
  private NetworkTableEntry nt_numApriltags;
  private NetworkTableEntry distanceToTargetTag;
  // private NetworkTableEntry nt_botpose; //todo: merge in helpers_util

  private double x;
  private double filteredX;
  private double y;
  private double area; // area is between 0 and 100. Calculated as a percentage of image
  private boolean targetValid;
  private boolean ledStatus; // true = ON
  private double filteredArea;

  // botpose index keys
  static final int X = 0;
  static final int Y = 1;
  static final int Z = 2;
  static final int RX = 3;
  static final int RY = 4;
  static final int RZ = 5;

  private long pipeline;

  @SuppressWarnings("unused")
  private LinearFilter x_iir;
  @SuppressWarnings("unused")
  private LinearFilter area_iir;
  public final String NT_Name = "Vision"; // expose data under DriveTrain table
  final String NT_Shooter_Name = "Shooter";
  private double filterTC = 0.08; // seconds, 2Hz cutoff T = 1/(2pi*f) was .2hz T=.8
  private int log_counter = 0;

  // generalize April Tag Target
  private int targetID = -1;
  private Translation2d targetTag = null;

  // private Pose2d megaPose;
  private Pose2d teamPose = new Pose2d(); // todo hack inits to avoid NPE 4/8/2023
  private Pose2d bluePose = new Pose2d();
  final private String LL_NAME = "";// "limelight" for if left blank
  private int numAprilTags;
  private double visionTimestamp;

  public Limelight() {
    x_iir = LinearFilter.singlePoleIIR(filterTC, Constants.DT);
    area_iir = LinearFilter.singlePoleIIR(filterTC, Constants.DT);
    table = NetworkTableInstance.getDefault().getTable("limelight");
    outputTable = NetworkTableInstance.getDefault().getTable(NT_Name);

    // these are "input" entries, to pull data from LL only
    //tv = table.getEntry("tv"); // target validity (1 or 0)
    leds = table.getEntry("ledMode");
    booleanLeds = table.getEntry("booleanLeds");
    pipelineNTE = table.getEntry("pipeline");

    // these are "output" entries for user debugging
    nt_bluepose_x = outputTable.getEntry("/LL Blue Pose X");
    nt_bluepose_y = outputTable.getEntry("/LL Blue Pose Y");
    nt_numApriltags = outputTable.getEntry("/LL_Num_Apriltag");
    NT_hasTarget = outputTable.getEntry("/LL hasTarget");
    outputTv = outputTable.getEntry("/Limelight Valid");
    outputTx = outputTable.getEntry("/Limelight X error");
    distanceToTargetTag = outputTable.getEntry("/Distance To TargetTag");
    disableLED();
  }

  /*
   * LL pose estimate may need a starting point to work from.
   * This matches PhotonVision and will be called anytime
   * the drivetrain's pose is reset. See swerverDrivetrain.java.
   */
  public void setInitialPose(Pose2d pose, double time) {

  }

  public void setTarget(int id, Translation2d location ) {
    targetID = id;
    targetTag = location;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();

    pipeline = pipelineNTE.getInteger(0);

      // LL apriltags stuff
      LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
      numAprilTags = llresults.targetingResults.targets_Fiducials.length;
      nt_numApriltags.setInteger(numAprilTags);
      visionTimestamp = Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline(LL_NAME) / 1000.0)
          - (LimelightHelpers.getLatency_Capture(LL_NAME) / 1000.0);

      if (numAprilTags > 0) {
        bluePose = LimelightHelpers.getBotPose2d_wpiBlue(LL_NAME);
        teamPose = LimelightHelpers.getBotPose2d_wpiBlue(LL_NAME); // assume/default blue for now

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red)
          //aliance info exists AND is red
          teamPose = LimelightHelpers.getBotPose2d_wpiRed(LL_NAME);
      }
    }
  

  public double getVisionTimestamp() {
    return visionTimestamp;
  }

  public Pose2d getBluePose() {
    return bluePose;
  }

  public Pose2d getTeamPose() {
    return teamPose;
  }

  public int getNumApriltags() {
    return numAprilTags;
  }

  public boolean hasAprilTarget() {
    return getNumApriltags() > 0;
  }

  public double getX() {
    return x;
  }

  public double getFilteredX() {
    return filteredX;
  }
  
  public double[] getAprilTagID(){
    LimelightTarget_Fiducial[] apriltag = 
    LimelightHelpers.getLatestResults("").targetingResults.targets_Fiducials;
    double[] tagIDs = new double[apriltag.length];
    for(int i = 0; i < apriltag.length; i++){
      tagIDs[i] = apriltag[i].fiducialID;
    }
    return tagIDs;
  }
  public double getTA(){
    return LimelightHelpers.getTA("");
  }
  public LimelightTarget_Fiducial[] getAprilTagsFromHelper(){
    return LimelightHelpers.getLatestResults("").targetingResults.targets_Fiducials;
  }

  public double getFilteredArea() {
    return filteredArea;
  }

  public double getY() {
    return y;
  }

  public double getArea() {
    return area;
  }

  public boolean getTarget() {
    return targetValid;
  }

  public boolean getLEDStatus() {
    return ledStatus;
  }

  public void disableLED() {
    leds.setNumber(1);
    ledStatus = false;
    booleanLeds.setBoolean(ledStatus);
  }

  public void enableLED() {
    leds.setNumber(3);
    ledStatus = true;
    booleanLeds.setBoolean(ledStatus);
  }

  public void toggleLED() {
    if (ledStatus) {
      disableLED();
    } else {
      enableLED();
    }

  }

  public void setPipeline(int pipe) {
    LimelightHelpers.setPipelineIndex(LL_NAME, pipe);
    if (pipe == 1) {
      enableLED();
    } else
      disableLED();
  }

  // switch between pipeline 0 and 1
  public void togglePipeline() {
    long pipe = pipelineNTE.getInteger(0);
    if (pipe == 0) {
      setPipeline(1);
      pipeline = 1;
    } else {
      setPipeline(0);
      pipeline = 0;
    }
  }

  public long getPipeline() {
    return pipeline;
  }

  public boolean valid() {
    return targetValid;  //TODO doesn't seem like this was set right -dpl
  }

  public void log() {
    log_counter++;
    if (log_counter % 20 == 0) {
      NT_hasTarget.setBoolean(targetValid);

      if (bluePose != null) {
        nt_bluepose_x.setDouble(bluePose.getX());
        nt_bluepose_y.setDouble(bluePose.getY());
      }

      outputTv.setValue(targetValid);
      outputTx.setDouble(x);


      if (targetTag != null  && targetID > 0) {
        distanceToTargetTag.setDouble(RobotContainer.getSubsystem(SwerveDrivetrain.class).getDistanceToTranslation(targetTag));
      }
    }
  }
}
