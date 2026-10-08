// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.subsystem.OdometryInterface;
import frc.robot2026.Constants.TheField;

// WARNING WARNING WARNING
// Ok, this code is only intended to be used with the autoClimberCommand sequence. It has no requirements and should not be touched by anything else. 
// Consider yourself warned

public class climberManuver extends Command {
  final static double clof = 0.08; // CLIMBER OFFSET FROM CENTER

  final Transform2d negMove = new Transform2d(new Translation2d( 1.0*(autoClimberCommand.BL * 0.5 + clof), 0.0),
      Rotation2d.kZero); // These values are 0 as we do not rotate on the ending move
  final Transform2d posMove = new Transform2d(new Translation2d(autoClimberCommand.BL * 0.5 + clof, 0.0),
      Rotation2d.kZero);

  final Pose3d blueCenter; // center of climber via tag
  final Pose3d redCenter;
  final boolean leftSide; // doing left or right

  // vars completed in initialize()
  private Rotation2d endRot;
  private Pose2d realCenter;

  PathPlannerPath path;
  Command runPath;

  final OdometryInterface odo;

  public climberManuver(boolean leftSide) {
    // decode climber related tags to get coordinates
    Optional<Pose3d> BlueCenter = TheField.fieldLayout.getTagPose(31);
    Optional<Pose3d> RedCenter = TheField.fieldLayout.getTagPose(15);
    blueCenter = (BlueCenter.isPresent()) ? BlueCenter.get() : null;
    redCenter = (RedCenter.isPresent()) ? RedCenter.get() : null;
    OdometryInterface tempOdo = RobotContainer.getSubsystemOrNull("vision_odo");
    this.odo = (tempOdo != null) ? tempOdo : RobotContainer.getSubsystem("odometry");

    this.leftSide = leftSide;

    // @Gavin - I don't think you need heading(), you will spec the endPose you want
    // also, if you did need it, it would be pulled in initialize(), not
    // construction.
    // try {
    // IHeadingProvider sensor = RobotContainer.getSubsystem("sensors");
    // endRot = sensor.getHeading();
    // } catch (Exception e) {
    // System.out.println("Current bot does not have a sensor. THE HELL ARE YOU
    // DOING HOW DID YOU GET HERE");
    // endRot = Rotation2d.fromDegrees((leftSide ? 0.0 : 180.0)); // THIS IS SO BAD
    // }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // @Gavin, you really can't check alliance during construction, but it is safe
    // at initialize()
    Alliance alliance = DriverStation.getAlliance().get();
    realCenter = (alliance == Alliance.Blue) ? blueCenter.toPose2d() : redCenter.toPose2d();
    // our computed waypoints
    //Rotation2d allienceOffset = (alliance == Alliance.Blue) ? Rotation2d.k180deg : Rotation2d.kZero;
    Rotation2d sideRotation; // for endRot based on tag & left/rt side
    Pose2d startPose;
    Pose2d endPose;
    Pose2d odoPose = odo.getPose();
    sideRotation =  Rotation2d.kZero;   // odoPose.getRotation(); //Current rotation is correct from AutoClimberCommand
    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not
    // use holonomic rotation.

    if (leftSide) {
      sideRotation =  Rotation2d.kZero;
      startPose = realCenter
          .transformBy(new Transform2d
            (new Translation2d(autoClimberCommand.TL - autoClimberCommand.BL * 0.5 - clof, //X
              (autoClimberCommand.BW + autoClimberCommand.TW) * 0.5),               //Y
               sideRotation));                                                      //Rotation
      endPose = startPose.transformBy(negMove);
    } else {
      //right side
      sideRotation =  Rotation2d.k180deg;
      startPose = realCenter
          .transformBy(new Transform2d(new Translation2d(autoClimberCommand.TL + autoClimberCommand.BL * 0.5  + clof, //X
              -0.5 * (autoClimberCommand.BW + autoClimberCommand.TW) ),                              //Y
              sideRotation));                                                                        //Rotation
      endPose = startPose.transformBy(posMove);
    }
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(odoPose, startPose, endPose);

    // endRot can be based on the heading of the tag and the Left/right side
    Rotation2d tagHeading = realCenter.getRotation();
    System.out.println("tagHeading for climb = " + tagHeading.toString());
    endRot = odoPose.getRotation();  //should be good already    //tagHeading.rotateBy(sideRotation);
    System.out.println("endRot for climb = " + endRot.toString());

    RobotLimits limits = RobotContainer.getRobotSpecs().getRobotLimits();
    /*
     * PathConstraints constraints = new PathConstraints(
     * limits.kMaxSpeed, limits.kMaxSpeed / 1.33,
     * limits.kMaxAngularSpeed, limits.kMaxAngularSpeed / 0.75);
     */

    PathConstraints constraints = new PathConstraints(
        1.0, limits.kMaxSpeed / 1.33,
        limits.kMaxAngularSpeed, limits.kMaxAngularSpeed / 0.75);
    // pulled from the MoveToPose Command
    // You can also use unlimited constraints, only limited by motor torque and
    // nominal battery voltage

    // Create the path using the waypoints created above
    path = new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can
              // be null for on-the-fly paths.
        new GoalEndState(0.0, endRot) // Goal end state. You can set a holonomic rotation here. If
    );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;
    runPath = AutoBuilder.followPath(path);
    runPath.initialize();
  }

  public void execute() {
    runPath.execute();
  }

  public boolean isFinished() {
    return runPath.isFinished();
  }

  public void end(boolean interrupted) {
    runPath.end(interrupted);
  }
}
