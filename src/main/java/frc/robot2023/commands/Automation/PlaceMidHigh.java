// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2023.commands.Automation;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.base.Constants;
import frc.base.RobotContainerOrig;
import frc.base.Constants.HorizontalScoringBlock;
import frc.base.Constants.HorizontalScoringSubstation;
import frc.base.Constants.VerticalScoringLane;
import frc.robot2023.commands.Arm.ArmLockForDrivingFS;
import frc.robot2023.commands.Arm.CollectivePositions;
import frc.robot2023.commands.Arm.ElbowMoveTo;
import frc.robot2023.commands.EndEffector.WheelsOut;
import frc.robot2023.commands.auto.goToScoringPosition;
import frc.robot2023.commands.auto.moveToPoint;
import frc.robot2023.commands.swerve.VelocityMove;
import frc.robot2023.subsystems.Claw_Substyem;
import frc.robot2023.subsystems.ColorSensors;
import frc.robot2023.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.util.DynamicSCG;

public class PlaceMidHigh extends SequentialCommandGroup {

  // SSs
  private HID_Xbox_Subsystem dc = RobotContainerOrig.RC().dc;
  public ColorSensors colorSensors = RobotContainerOrig.RC().colorSensors;
  private Claw_Substyem claw = RobotContainerOrig.RC().claw;

  // constants
  final double DEADZONE2 = 0.025; // [percent^2] this number is squared!!
  final double SPEED_MOVE = 0.5; // [m/s] speed moving to / from target
  final double TIME_MOVE = 1.0; // [s] time to move to / from target
  final double TIME_DROP = 0.5; // [s] time to wait after claw opens / wheels spin out before going back

  // state vars
  private HorizontalScoringBlock horizontalRequest;
  private HorizontalScoringSubstation substationRequest;
  private VerticalScoringLane verticalRequest;
  private Pose2d goalPose;

  /**
   * This class is a DynamicSCG.
   * 
   * Constructs and schedules a new Place command. Encompasses sdt movement, arm
   * extension / retraction.
   * Assumes piece is already in claw (this should be done upon picking up).
   * 
   * @param horizontalRequest The station (macro-level) request
   * @param substationRequest The substation (micro-level) request
   * @param verticalRequest The height
   */
  public PlaceMidHigh(HorizontalScoringBlock horizontalRequest, 
                      HorizontalScoringSubstation substationRequest, 
                      VerticalScoringLane verticalRequest) {
    this.horizontalRequest = horizontalRequest;
    this.substationRequest = substationRequest;
    this.verticalRequest = verticalRequest;
  }

  // Called when the command is initially scheduled.
  @Override
  public void doFirstOnInit() {
    goalPose = calculateTargetPose();
    // 1. Move to safe location for arm extension 
    move();

    // 2. Move arm out and drop, then wait 1sec
    switch (substationRequest) {
      case Center:
        Cube();
        break;
      default:
        Cone();
        break;
    }

    // 3. Move back and retract arm to travel position
    Retract();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinishedCondition() {
    return dc.rightStickMotionDriver();
  }

  /**
   * Adds sdt movement to target node.
   */
  private void move() {
    // 1. move to general vicinity
    this.addCommands(new goToScoringPosition(new PathConstraints(2, 3), horizontalRequest, substationRequest));
  }

  /**
   * Constructs pacing this based on object being a cone facing forward.
   */
  private void Cone() {
    switch (verticalRequest) {
      case High:
        this.addCommands(new PlaceTele(CollectivePositions.placeConeHighFS));
        break;
      case Middle:
        this.addCommands(new PlaceTele(CollectivePositions.placeConeMidFS));
        break;
      default:
        break;
    }

    this.addCommands(
      new ElbowMoveTo(95.0, 60.0), // lower to dropping position
      new InstantCommand(() -> { claw.open();}).andThen(new WaitCommand(TIME_DROP)));
  }

  /**
   * Constructs placing this based on object being a cube.
   * Coming out of travel position.
   */
  private void Cube() {
    switch (verticalRequest) {
      case High:
        this.addCommands(new PlaceTele(CollectivePositions.placeConeHighFS));
        break;
      case Middle:
        this.addCommands(new PlaceTele(CollectivePositions.placeConeMidFS));
        break;
      default:
        break;
    }

    this.addCommands(
      new WheelsOut().withTimeout(TIME_DROP));  //wait for claw to open and cone drop
  }

  /**
   * Retracts piece
   */
  private void Retract() {
    Pose2d retractPose;
    double distance = 0.6; //how far back to move (m)

    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) { //BLUE ALLIANCE
      retractPose = new Pose2d(goalPose.getX() + distance, goalPose.getY(), goalPose.getRotation()); //distance away from scoring station, blue side
    }
    else{
      retractPose = new Pose2d(goalPose.getX() - distance, goalPose.getY(), goalPose.getRotation()); //distance away from scoring station, red side
    }

    this.addCommands(
      new ElbowMoveTo(145.0), //return to high position to avoid low post
      new ParallelCommandGroup(
        new PrintCommand("Starting Parallel group + derek getting pissed"),
        new VelocityMove(-0.5, 0.0, 1.0),
        new PrintCommand("it's a race!"),
        //no worky 2+ new moveToPoint(new PathConstraints(1.0, 1.0), retractPose), //move slowly back while retracting arm
        new SequentialCommandGroup(
          new WaitCommand(3.0), //let the move start first for 0.5s so arm doesn't catch low pole
          new ArmLockForDrivingFS() //then start to retract arm
        )
      )
      );
  }

  //calculate which constant scoring pose is appropriate goal
  private Pose2d calculateTargetPose(){
      //FOR BLUE: 2 for left (driver's point of view), 1 for center, 0 for right
      HorizontalScoringBlock HorizontalScoringBlock = this.horizontalRequest;
      HorizontalScoringSubstation HorizontalScoringSubstation = this.substationRequest;
      int scoringBlock; 
      int scoringAdjusted;
      Pose2d targetPose;

    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) { //BLUE ALLIANCE


      if(HorizontalScoringSubstation.equals(HorizontalScoringSubstation.Left)) scoringBlock = 2;
      else if(HorizontalScoringSubstation.equals(HorizontalScoringSubstation.Right)) scoringBlock = 0;
      else scoringBlock = 1;

      //FOR BLUE: left is largest index of scoring trio
      switch(HorizontalScoringBlock){
        case Left:
          scoringAdjusted = 2;
          break;
        case Center:
          scoringAdjusted = 1;
          break;
        default:
        case Right:
          scoringAdjusted = 0;
          break;      
      }
      targetPose = frc.base.FieldPoses.blueScorePoses[scoringBlock][scoringAdjusted];
    }
    else { //RED ALLIANCE
      //FOR RED: 0 for left (driver's point of view), 1 for center, 2 for right
      if(HorizontalScoringSubstation.equals(HorizontalScoringSubstation.Left)) scoringBlock = 0;
      else if(HorizontalScoringSubstation.equals(HorizontalScoringSubstation.Right)) scoringBlock = 2;
      else scoringBlock = 1;

      //FOR RED: left is smallest index of scoring trio
      switch(HorizontalScoringBlock){
        case Left:
          scoringAdjusted = 0;
          break;
        case Center:
          scoringAdjusted = 1;
          break;
        default:
        case Right:
          scoringAdjusted = 2;
          break;      
      }
      targetPose = frc.base.FieldPoses.blueScorePoses[scoringBlock][scoringAdjusted];
    }
   return targetPose;
  }

  @Override
  public void doFirstOnEnd() {
    RobotContainerOrig.RC().drivetrain.enableVisionPose();
    RobotContainerOrig.RC().drivetrain.disableVisionPoseRotation();
  }

}